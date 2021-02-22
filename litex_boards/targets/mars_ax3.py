#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen import *
from migen.genlib.misc import WaitTimer


from litex_boards.platforms import mars_ax3
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict
from litex.build.io import DifferentialOutput

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.cores.spi_flash import S7SPIFlash
from litex.soc.cores.icap import ICAP
from litex.soc.interconnect.csr import *

#from litedram.modules import NT5CC128M16
from litedram.modules import DDR3Module, _TechnologyTimings, _SpeedgradeTimings
from litedram.phy import s7ddrphy

from liteeth.phy.s7rgmii import LiteEthPHYRGMII

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


# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys       = ClockDomain()
        self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_idelay    = ClockDomain()
        self.clock_domains.cd_dac       = ClockDomain(reset_less=True)
        self.clock_domains.cd_dac_180   = ClockDomain(reset_less=True)
        #self.clock_domains.cd_eth       = ClockDomain()

        # # #
        ## TODO recheck these, changed from 100 MHz to 50MHz  arty to mars ax3
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset") | self.rst)
        pll.register_clkin(platform.request("clk50"), 50e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay,    200e6)
        pll.create_clkout(self.cd_dac,       100e6)
        pll.create_clkout(self.cd_dac_180,   100e6, phase=180)

        #pll.create_clkout(self.cd_eth,       25e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        #self.comb += platform.request("eth_ref_clk").eq(self.cd_eth.clk)


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
        '''


        #sync = getattr(self.sync, "sys")
        self.sync.sys4x += [
            If(counter > self.freq_bound.storage,
                counter.eq(0),
                data_val.eq(data_val + 10)
            ).Else(
                counter.eq(counter + 1)
            )
        ]
        self.comb += data_a.eq(data_val),
        '''

        self.sync.sys += [
            If(counter > 1023,
                counter.eq(0),
            ).Else(
                counter.eq(counter + 8)
            )
        ]
        self.comb += data_a.eq(counter),


        '''
        # chaser    = Signal(10)

        counter = Signal(32, reset_less=True)

        sync = getattr(self.sync, clock_domain)
        sync += [
            If(self.enable,
               counter.eq(counter + 1),
               If(counter < self.width,
                  pwm.eq(1)
                  ).Else(
                   pwm.eq(0)
               ),
               If(counter >= (self.period - 1),
                  counter.eq(0)
                  )
               ).Else(
                counter.eq(0),
                pwm.eq(0)
            )

        '''
        '''
        Instance("dac",
                 i_clk=ClockSignal(),
                 i_rst=ResetSignal(),
                
                 )
        '''
        '''
                #(parameter IN_DATA_WIDTH=20)
        (
          // Global signals
          input wire i_clk,
          input wire i_reset,
        
          // AXI stream interface
          input wire [(IN_DATA_WIDTH-1):0] i_tdata,
          input wire i_tvalid,
          output reg o_tready = 0,
        
          // DAC interface
          output reg [DAC_DATA_WIDTH-1:0] o_sig_a,
          output reg [DAC_DATA_WIDTH-1:0] o_sig_b,
          output reg o_ncw
          );
        '''

        # # #
        '''
        n         = len(pads)
        chaser    = Signal(n)
        mode      = Signal(reset=_CHASER_MODE)
        timer     = WaitTimer(int(period*sys_clk_freq/(2*n)))
        self.submodules += timer
        self.comb += timer.wait.eq(~timer.done)
        self.sync += If(timer.done, chaser.eq(Cat(~chaser[-1], chaser)))
        self.sync += If(self._out.re, mode.eq(_CONTROL_MODE))
        self.comb += [
            If(mode == _CONTROL_MODE,
                pads.eq(self._out.storage)
            ).Else(
                pads.eq(chaser)
            )
        ]
        '''

# litedram struct, used here as it is not merged and tested for upstream ---------------------------
# NICK DEBUG  NT5CC128M16IP-DII
class NT5CC128M16(DDR3Module):
    memtype = "DDR3"
    # geometry
    nbanks = 8
    nrows  = 16384
    ncols  = 1024
    # timings
    technology_timings = _TechnologyTimings(tREFI=64e6/8192, tWTR=(4, 7.5), tCCD=(4, None), tRRD=(4, 10), tZQCS=(64, 80))
    speedgrade_timings = {
        "800" : _SpeedgradeTimings(tRP=15,     tRCD=15,     tWR=15, tRFC=(None, 260), tFAW=(None, 40), tRAS=37.5),
        "1066": _SpeedgradeTimings(tRP=15,     tRCD=15,     tWR=15, tRFC=(None, 260), tFAW=(None, 40), tRAS=37.5),
        "1333": _SpeedgradeTimings(tRP=13.125, tRCD=13.125, tWR=15, tRFC=(None, 260), tFAW=(None, 30), tRAS=36),
        "1600": _SpeedgradeTimings(tRP=13.125, tRCD=13.125, tWR=15, tRFC=(None, 260), tFAW=(None, 30), tRAS=35),
        "1866": _SpeedgradeTimings(tRP=13.125, tRCD=13.125, tWR=15, tRFC=(None, 260), tFAW=(None, 27), tRAS=34),
    }
    speedgrade_timings["default"] = speedgrade_timings["800"]
    #            <Parameters twtr="7.5" trrd="7.5" trefi="7.8" tfaw="40" trtp="7.5" tcke="5.625" trfc="160" trp="13.75" tras="35" trcd="13.75" />

# NICK DEBUG

# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    mem_map = {**SoCCore.mem_map, **{"spiflash": 0x20000000}}
    def __init__(self, toolchain="vivado", spiflash="spiflash_1x", sys_clk_freq=int(100e6), with_ethernet=False, with_etherbone=False, eth_ip="192.168.1.50", ident_version=True, **kwargs):
        platform = mars_ax3.Platform(toolchain=toolchain)

        # Increasing the integration rom size as we would like to enable SPI mode SDcard in bios
        kwargs["integrated_rom_size"] = 0x10000

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on Mars AX3 (IQFPGA) AAUAST6",
            ident_version  = ident_version,
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"),
                memtype        = "DDR3",
                nphases        = 4,
                sys_clk_freq   = sys_clk_freq)
            self.add_csr("ddrphy")
            self.add_sdram("sdram",
                phy                     = self.ddrphy,
                module                  = NT5CC128M16(sys_clk_freq, "1:4"),
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = True,
                with_bist               = True
            )

        # Add debug interface if the CPU has one ---------------------------------------------------
        if hasattr(self.cpu, "debug_bus"):
            self.register_mem(
                name="vexriscv_debug",
                address=0xf00f0000,
                interface=self.cpu.debug_bus,
                size=0x100)

        # Memory mapped SPI Flash ------------------------------------------------------------------

        spiflash_pads = platform.request(spiflash)
        spiflash_pads.clk = Signal()
        self.specials += Instance(
            "STARTUPE2",
            i_CLK=0, i_GSR=0, i_GTS=0, i_KEYCLEARB=0, i_PACK=0,
            i_USRCCLKO=spiflash_pads.clk, i_USRCCLKTS=0, i_USRDONEO=1, i_USRDONETS=1)
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

        # Mars AX3 specific stuff
        #self.comb += platform.request("ddr3_vsel").eq(0) # only for LV RAM operation

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)
        self.add_csr("leds")

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
        #self.comb += dac_plat.clkx_test.eq(self.crg.cd_dac.clk)
        #self.comb += dac_plat.clkx_test2.eq(self.crg.cd_dac_180.clk)


        '''
        clk_pin = Signal()
        self.specials += MultiReg(pads.clk, clk_pin)
        clk_d = Signal()
        self.sync += clk_d.eq(clk_pin)
        rising_edge  = Signal()
        falling_edge = Signal()
        self.comb += [rising_edge.eq(clk_pin & ~clk_d), falling_edge.eq(~clk_pin & clk_d)]
        '''

        # ICAP -------------------------------------------------------------------------------------
        self.submodules.icap = ICAP(platform)
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)
        self.add_csr("icap")
        #ICAP can be used for botting different bitstreams from flash -- I think
        #TODO add wb test scripts from netv2?


        self.add_jtagbone()

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            self.submodules.ethphy = LiteEthPHYRGMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"))
            self.add_csr("ethphy")
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip)


        '''
        AXIStreamInterface

        LiteDRAMCrossbar

        axi.AXIStreamInterface(
            data_width=32,

        )

        # Zynq7000 Integration ---------------------------------------------------------------------
        if kwargs.get("cpu_type", None) == "zynq7000":
            # Get and set the pre-generated .xci FIXME: change location? add it to the repository?
            os.system("wget https://kmf2.trabucayre.com/redpitaya_ps7.txt")
            os.makedirs("xci", exist_ok=True)
            os.system("cp redpitaya_ps7.txt xci/redpitaya_ps7.xci")
            self.cpu.set_ps7_xci("xci/redpitaya_ps7.xci")

            # Connect AXI GP0 to the SoC with base address of 0x43c00000 (default one)
            wb_gp0  = wishbone.Interface()
            self.submodules += axi.AXI2Wishbone(
                axi          = self.cpu.add_axi_gp_master(),
                wishbone     = wb_gp0,
                base_address = 0x43c00000)
            self.add_wb_master(wb_gp0)
            use_ps7_clk = True
            sys_clk_freq = 125e6
        '''

        '''
        user_port = self.sdram.crossbar.get_port()
        axi_port = LiteDRAMAXIPort(
            user_port.data_width,
            user_port.address_width + log2_int(user_port.data_width // 8),
        )
        axi2native = LiteDRAMAXI2Native(axi_port, user_port)
        self.submodules += axi2native
        '''
        #So this create the AXI port interface, and I can then connect that streaming thing on that?
        #that's the native width of the controller. You can verify it with: print(user_port.data_width), for a 16-bit DDR
        #16-bit DDR3 on Artix7 it will be 128-bit
        #256-bit for a 32-bit DDR3
        #print(f"NICK DEBUG {user_port.data_width}")
        '''
        self.submodules += LiteDRAMAXI2Native(
            axi=self.cpu.mem_axi,
            port=port,
            base_address=self.bus.regions["main_ram"].origin)
        '''


        #axistream = AXIStreamInterface(axi_port)

        '''
        user_port = self.sdram.crossbar.get_port()
        self.submodules.dma = _MyDMA(port=user_port, sys_clk_freq=sys_clk_freq)
        self.add_csr("dma")
        '''



# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on foo")
    parser.add_argument("--toolchain",        default="vivado",                 help="Toolchain use to build (default: vivado)")
    parser.add_argument("--build",            action="store_true",              help="Build bitstream")
    parser.add_argument("--load",             action="store_true",              help="Load bitstream")
    parser.add_argument("--sys-clk-freq",     default=100e6,                    help="System clock frequency (default: 50MHz)")
    parser.add_argument("--sim",              action="store_true",              help="My attempt to enable simulation")
    parser.add_argument("--trace",                action="store_true",     help="enable VCD tracing")
    parser.add_argument("--trace-start",          default=0,               help="cycle to start VCD tracing")
    parser.add_argument("--trace-end",            default=-1,              help="cycle to end VCD tracing")
    parser.add_argument("--opt-level",            default="O3",            help="compilation optimization level")
    ethopts = parser.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",   action="store_true",              help="Enable Ethernet support")
    ethopts.add_argument("--with-etherbone",  action="store_true",              help="Enable Etherbone support")
    parser.add_argument("--eth-ip",           default="192.168.1.50", type=str, help="Ethernet/Etherbone IP address")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",  action="store_true",              help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",      action="store_true",              help="Enable SDCard support")
    parser.add_argument("--no-ident-version", action="store_false",             help="Disable build time output")
    builder_args(parser)
    soc_sdram_args(parser)
    #vivado_build_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(
        toolchain=args.toolchain,
        sys_clk_freq=int(float(args.sys_clk_freq)),
        with_ethernet=args.with_ethernet,
        with_etherbone=args.with_etherbone,
        eth_ip=args.eth_ip,
        ident_version=args.no_ident_version,
        **soc_sdram_argdict(args)
    )
    # soc.platform.add_extension(arty._sdcard_pmod_io)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()


    if not args.sim:
        print("NICK doing REAL SoC stuff")
        # Real SoC stuff -------------------------------------------------------------------------------
        builder = Builder(soc, **builder_argdict(args))
        builder_kwargs = vivado_build_argdict(args) if args.toolchain == "vivado" else {}
        builder.build(**builder_kwargs, run=args.build)

        if args.load:
            prog = soc.platform.create_programmer()
            prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    else:
        print("NICK doing SIMULATION SoC stuff")
        # Simulation stuff -----------------------------------------------------------------------------
        sim_config = SimConfig(default_clk="sys_clk")
        sim_config.add_module("serial2console", "serial")
        if args.with_ethernet:
            sim_config.add_module("ethernet", "eth", args={"interface": "tap0", "ip": args.remote_ip})

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
