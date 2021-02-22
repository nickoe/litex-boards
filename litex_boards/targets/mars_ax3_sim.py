#!/usr/bin/env python3
import os
import argparse

from migen import *
from migen.genlib.misc import WaitTimer

#from litex_boards.platforms import mars_ax3
from litex.build.io import DifferentialOutput

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.cores.spi_flash import S7SPIFlash
from litex.soc.interconnect.csr import *

from litedram.phy import s7ddrphy

from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone
from litedram.frontend.axi import *

from litex.soc.cores.dma import WishboneDMAWriter, WishboneDMAReader
from litedram.frontend.dma import LiteDRAMDMAWriter, LiteDRAMDMAReader
from litedram.common import LiteDRAMNativePort
from litedram.frontend.axi import LiteDRAMAXIPort


from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litedram import modules as litedram_modules
from litedram.phy.model import SDRAMPHYModel
from litex.tools.litex_sim import sdram_module_nphases, get_sdram_phy_settings
from litedram.core.controller import ControllerSettings



def get_ashift_awidth(dram_port):
    if isinstance(dram_port, LiteDRAMNativePort):
        ashift = log2_int(dram_port.data_width//8)
        awidth = dram_port.address_width + ashift
    elif isinstance(dram_port, LiteDRAMAXIPort):
        ashift = log2_int(dram_port.data_width//8)
        awidth = dram_port.address_width
    else:
        raise NotImplementedError
    return ashift, awidth

class DMAReaderDriver:
    def __init__(self, dma):
        self.dma  = dma
        self.data = []

    def read(self, address_list):
        n_last = len(self.data)
        yield self.dma.sink.valid.eq(1)
        for adr in address_list:
            yield self.dma.sink.address.eq(adr)
            while not (yield self.dma.sink.ready):
                yield
            while (yield self.dma.sink.ready):
                yield
        yield self.dma.sink.valid.eq(0)
        while len(self.data) < n_last + len(address_list):
            yield

    @passive
    def read_handler(self):
        yield self.dma.source.ready.eq(1)
        while True:
            if (yield self.dma.source.valid):
                self.data.append((yield self.dma.source.data))
            yield


class _MyDMA(Module, AutoCSR):
    def __init__(self, port, sys_clk_freq, period=1e0):
        ashift, awidth = get_ashift_awidth(port)
        self.start       = Signal()
        self.done        = Signal()
        self.base        = Signal(awidth)
        self.end         = Signal(awidth)
        self.length      = Signal(awidth)
        self.ticks = Signal(64)

        # DMA --------------------------------------------------------------------------------------
        dma = LiteDRAMDMAReader(port)
        self.submodules += dma

        # Address FSM ------------------------------------------------------------------------------
        cmd_counter = Signal(port.address_width, reset_less=True)

        #NOTE Right now we just dump 1kB of IQ data at 0x41000000 with boot.json.
        #TODO use a fsm, similar to _LiteDRAMBISTChecker or _LiteDRAMBISTGenerator to excercise the address and map it to the registers wired to the DAC?
        # I mean (dac_plat.data_a and dac_plat.data_b) instead of using the _MyDAC module
        #TODO Secondly this should output in an AXIStreamInterface

        self.data_iq_addr = CSRStorage(32, description="Address of our IQ samples in memory", write_from_dev=True)
        self.data_iq = CSRStorage(32, fields=[
                                   CSRField("i", description="Data I", size=10),
                                   CSRField("q", description="Data Q", size=10),
                               ], description="IQ sample",)
        #data_a.eq(self.data_iq_storage.storage[0:10])
        #data_b.eq(self.data_iq_storage.storage[10:20])

        if isinstance(port, LiteDRAMNativePort): # addressing in dwords
            dma_sink_addr = dma.sink.address
        else:
            raise NotImplementedError

        dma_sink_addr

        # Data / Address FSM -----------------------------------------------------------------------
        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(self.start,
                NextValue(self.data_iq_addr, 0x41000000),
                NextState("RUN")
            ),
            NextValue(self.ticks, 0)
        )
        fsm.act("WAIT",
            If(self.run_cascade_in,
                NextState("RUN")
            )
        )
        fsm.act("RUN",
            dma.sink.valid.eq(1),
            If(dma.sink.ready,
                NextValue(self.data_iq_addr, self.data_iq_addr + 1),
                If(self.data_iq_addr == (0x41000000 + 1024),
                    NextState("DONE")
                ).Elif(~self.run_cascade_in,
                    NextState("WAIT")
                )
            ),
            NextValue(self.ticks, self.ticks + 1)
        )
        fsm.act("DONE",
            self.run_cascade_out.eq(1),
            self.done.eq(1)
        )

        self.comb += [
            dma_sink_addr.eq(addr_port.dat_r),
            dma.sink.data.eq(data_port.dat_r),
        ]



class _MyDAC(Module, AutoCSR):
    def __init__(self, data_a, data_b, cw, sys_clk_freq, period=1e0):
        self.cw = CSRStorage(1, description="DAC code word register")
        self.data_a_storage = CSRStorage(10, fields=[
                                   CSRField("a", description="Data A", size=10),
                               ], description="DAC data register",)
        self.data_b_storage = CSRStorage(10, fields=[
                                   CSRField("b", description="Data B", size=10),
                               ], description="DAC data register",)
        self.comb += [
            #data_a.eq(self.data_a_storage.storage[0:10]),
            data_b.eq(self.data_b_storage.storage[0:10]),
            cw.eq(self.cw.storage)
        ]

        self.freq_bound = CSRStorage(32)
        counter = Signal(24) # counter at sys clk
        data_val = Signal(10) # value for DAC

        self.sync.sys += [
            If(counter > 1023,
                counter.eq(0),
            ).Else(
                counter.eq(counter + 8)
            )
        ]
        self.comb += data_a.eq(counter),

# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data",  Pins(8)),
    ),
    ("serial", 1,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data", Pins(8)),

        Subsignal("sink_valid", Pins(1)),
        Subsignal("sink_ready", Pins(1)),
        Subsignal("sink_data", Pins(8)),
     ),
    ("user_led", 0, Pins(1)),
    ("user_led", 1, Pins(2)),
    ("user_led", 2, Pins(3)),
    ("user_led", 3, Pins(4)),
    ("spisdcard", 0,
        Subsignal("clk", Pins(1)),
        Subsignal("mosi", Pins(1), Misc("PULLUP True")),
        Subsignal("cs_n", Pins(1), Misc("PULLUP True")),
        Subsignal("miso", Pins(1), Misc("PULLUP True")),
     ),
]


# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    csr_map = {**SoCCore.csr_map, **{
        "ctrl":   0,
        "uart":   2,
    }}
    mem_map = {**SoCCore.mem_map, **{"spiflash": 0x20000000}}
    def __init__(self, toolchain="vivado", spiflash="spiflash_1x", sys_clk_freq=int(100e6), ident_version=True,
                 init_memories=False,
                 sdram_module="MT48LC16M16",
                 sdram_data_width=32,
                 sdram_verbosity=0,
                 **kwargs):
        platform = Platform()

        # Increasing the integration rom size as we would like to enable SPI mode SDcard in bios
        kwargs["integrated_rom_size"] = 0x10000

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident          = "LiteX SoC on Mars AX3 (IQFPGA) AAUAST6",
            ident_version  = ident_version,
            #integrated_rom_size      = 0x10000,
            #uart_name = "sim",
            **kwargs)
        self.add_config("DISABLE_DELAYS")

        ram_init = []
        if init_memories:
            ram_init = get_mem_data({
                "./build/sim/software/bios/bios.bin": "0x00000000",
            }, "little")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # SDRAM ------------------------------------------------------------------------------------
        sdram_clk_freq   = int(100e6) # FIXME: use 100MHz timings
        sdram_module_cls = getattr(litedram_modules, sdram_module)
        sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
        sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
        phy_settings     = get_sdram_phy_settings(
            memtype    = sdram_module.memtype,
            data_width = sdram_data_width,
            clk_freq   = sdram_clk_freq)
        self.submodules.sdrphy = SDRAMPHYModel(
            module    = sdram_module,
            settings  = phy_settings,
            clk_freq  = sdram_clk_freq,
            verbosity = sdram_verbosity,
            init      = ram_init)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = sdram_module,
            origin        = self.mem_map["main_ram"],
            l2_cache_size = 0)
        self.add_constant("SDRAM_TEST_DISABLE") # Skip SDRAM test to avoid corrupting pre-initialized contents.

        # Add debug interface if the CPU has one ---------------------------------------------------
        if hasattr(self.cpu, "debug_bus"):
            self.register_mem(
                name="vexriscv_debug",
                address=0xf00f0000,
                interface=self.cpu.debug_bus,
                size=0x100)

        # Memory mapped SPI Flash ------------------------------------------------------------------
        '''
        spiflash_pads = platform.request(spiflash)
        spiflash_pads.clk = Signal()

        spiflash_dummy = {
            "spiflash_1x": 8,
            "spiflash_4x": 11,
        }
        self.submodules.spiflash = SpiFlash(
            spiflash_pads,
            dummy=spiflash_dummy[spiflash],
            div=platform.spiflash_clock_div,
            endianness=self.cpu.endianness)
        self.add_csr("spiflash")
        self.add_constant("SPIFLASH_PAGE_SIZE", platform.spiflash_page_size)
        self.add_constant("SPIFLASH_SECTOR_SIZE", platform.spiflash_sector_size)
        self.add_constant("SPIFLASH_TOTAL_SIZE", platform.spiflash_total_size)
        self.add_wb_slave(
            self.mem_map["spiflash"],
            self.spiflash.bus,
            platform.spiflash_total_size)
        self.add_memory_region(
            "spiflash",
            self.mem_map["spiflash"],
            platform.spiflash_total_size)
        '''
        # Mars AX3 specific stuff
        #self.comb += platform.request("ddr3_vsel").eq(0) # only for LV RAM operation

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)
        self.add_csr("leds")

        '''
        # MAX DAC...
        dac_plat = platform.request("dac")
        self.submodules.dac = _MyDAC(
            data_a=dac_plat.data_a,
            data_b=dac_plat.data_b,
            cw=dac_plat.cw,
            sys_clk_freq=sys_clk_freq)
        self.add_csr("dac")

        # clocking for the DAC
        # As the differential output is in the same bank as the other signals that are LVCMOS33 we
        # can't really drive this as a diff out. :(
        # And the logic inputs needs 0.64*DVdd = 2.145 V for high state..
        #self.specials += DifferentialOutput(self.crg.cd_dac.clk, dac_plat.clkx_p, dac_plat.clkx_n)
        self.comb += dac_plat.clkx_p.eq(self.crg.cd_dac.clk)
        self.comb += dac_plat.clkx_n.eq(self.crg.cd_dac_180.clk)
        '''

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on foo")
    parser.add_argument("--toolchain",        default="vivado",                 help="Toolchain use to build (default: vivado)")
    parser.add_argument("--build",            action="store_true",              help="Build bitstream")
    parser.add_argument("--load",             action="store_true",              help="Load bitstream")
    parser.add_argument("--sys-clk-freq",     default=100e6,                    help="System clock frequency (default: 50MHz)")
    parser.add_argument("--trace",                action="store_true",     help="enable VCD tracing")
    parser.add_argument("--trace-start",          default=0,               help="cycle to start VCD tracing")
    parser.add_argument("--trace-end",            default=-1,              help="cycle to end VCD tracing")
    parser.add_argument("--opt-level",            default="O3",            help="compilation optimization level")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",  action="store_true",              help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",      action="store_true",              help="Enable SDCard support")
    parser.add_argument("--no-ident-version", action="store_false",             help="Disable build time output")
    builder_args(parser)
    args = parser.parse_args()

    sim_config = SimConfig(default_clk="sys_clk")
    sim_config.add_module("serial2console", "serial")

    soc = BaseSoC(
        toolchain=args.toolchain,
        sys_clk_freq=int(float(args.sys_clk_freq)),
        ident_version=args.no_ident_version,
        **soc_sdram_argdict(args)
    )

    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()



    #verilator
    board_name = "sim"
    build_dir  = os.path.join("build", board_name)
    builder = Builder(soc, output_dir=build_dir,
        compile_gateware = args.build,
        csr_json         = os.path.join(build_dir, "csr.json"))
    builder.build(sim_config=sim_config,
        run         = args.build,
        opt_level   = args.opt_level,
        trace       = args.trace,
        trace_start = int(args.trace_start),
        trace_end   = int(args.trace_end))


if __name__ == "__main__":
    main()
