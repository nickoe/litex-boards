#!/usr/bin/env python3
import sys
import argparse

from migen import *

from litex.build.generic_platform import *
from litex.build.sim import SimPlatform
from litex.build.sim.config import SimConfig

from litex.soc.integration.common import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import *
from litex.soc.cores.bitbang import *
from litex.soc.cores.cpu import CPUS

from litedram import modules as litedram_modules
from litedram.modules import parse_spd_hexdump
from litedram.common import *
from litedram.phy.model import SDRAMPHYModel
from litedram.frontend.bist import get_ashift_awidth

from litex.soc.cores.dma import WishboneDMAWriter, WishboneDMAReader
from litedram.frontend.dma import LiteDRAMDMAWriter, LiteDRAMDMAReader
from litedram.common import LiteDRAMNativePort
from litedram.frontend.axi import LiteDRAMAXIPort

from litevideo.output.core import DMAReader

from liteeth.phy.model import LiteEthPHYModel
from liteeth.mac import LiteEthMAC
from liteeth.core.arp import LiteEthARP
from liteeth.core.ip import LiteEthIP
from liteeth.core.udp import LiteEthUDP
from liteeth.core.icmp import LiteEthICMP
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone
from liteeth.common import *

from litescope import LiteScopeAnalyzer

from litex.soc.cores.led import LedChaser


# IOs ----------------------------------------------------------------------------------------------

_io = [
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
    ("eth_clocks", 0,
        Subsignal("tx", Pins(1)),
        Subsignal("rx", Pins(1)),
    ),
    ("eth", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),
    ("user_led", 0, Pins(1)),
    ("user_led", 1, Pins(1)),
    ("user_led", 2, Pins(1)),
    ("user_led", 3, Pins(1)),
    ("dac", 0,
     Subsignal("data_a", Pins("R1 T1 U1 V1 U4 U3 U2 V2 V7 V6"), IOStandard("LVCMOS33")),
     Subsignal("data_b", Pins("N5 P5 L1 M1 N2 N1 P4 P3 R6 R5"), IOStandard("LVCMOS33")),
     Subsignal("cw", Pins("L3"), IOStandard("LVCMOS33")),
     Subsignal("clkx_p", Pins("K5"), IOStandard("LVCMOS33")),
     Subsignal("clkx_n", Pins("L4"), IOStandard("LVCMOS33")),
     Subsignal("clkx_test", Pins("F6"), IOStandard("LVCMOS33")),
     Subsignal("clkx_test2", Pins("G2"), IOStandard("LVCMOS33")),
     ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# DFI PHY model settings ---------------------------------------------------------------------------

sdram_module_nphases = {
    "SDR":   1,
    "DDR":   2,
    "LPDDR": 2,
    "DDR2":  2,
    "DDR3":  4,
    "DDR4":  4,
}

def get_sdram_phy_settings(memtype, data_width, clk_freq):
    nphases = sdram_module_nphases[memtype]

    if memtype == "SDR":
        # Settings from gensdrphy
        rdphase       = 0
        wrphase       = 0
        cl            = 2
        cwl           = None
        read_latency  = 4
        write_latency = 0
    elif memtype in ["DDR", "LPDDR"]:
        # Settings from s6ddrphy
        rdphase       = 0
        wrphase       = 1
        cl            = 3
        cwl           = None
        read_latency  = 5
        write_latency = 0
    elif memtype in ["DDR2", "DDR3"]:
        # Settings from s7ddrphy
        tck             = 2/(2*nphases*clk_freq)
        cl, cwl         = get_default_cl_cwl(memtype, tck)
        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)
        rdphase         = get_sys_phase(nphases, cl_sys_latency, cl)
        wrphase         = get_sys_phase(nphases, cwl_sys_latency, cwl)
        read_latency    = cl_sys_latency + 6
        write_latency   = cwl_sys_latency - 1
    elif memtype == "DDR4":
        # Settings from usddrphy
        tck             = 2/(2*nphases*clk_freq)
        cl, cwl         = get_default_cl_cwl(memtype, tck)
        cl_sys_latency  = get_sys_latency(nphases, cl)
        cwl_sys_latency = get_sys_latency(nphases, cwl)
        rdphase         = get_sys_phase(nphases, cl_sys_latency, cl)
        wrphase         = get_sys_phase(nphases, cwl_sys_latency, cwl)
        read_latency    = cl_sys_latency + 5
        write_latency   = cwl_sys_latency - 1

    sdram_phy_settings = {
        "nphases":       nphases,
        "rdphase":       rdphase,
        "wrphase":       wrphase,
        "cl":            cl,
        "cwl":           cwl,
        "read_latency":  read_latency,
        "write_latency": write_latency,
    }

    return PhySettings(
        phytype      = "SDRAMPHYModel",
        memtype      = memtype,
        databits     = data_width,
        dfi_databits = data_width if memtype == "SDR" else 2*data_width,
        **sdram_phy_settings,
    )

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
            data_a.eq(self.data_a_storage.storage[0:10]),
            data_b.eq(self.data_b_storage.storage[0:10]),
            cw.eq(self.cw.storage)
        ]


class _MyDMA(Module, AutoCSR):
    def __init__(self, port, sys_clk_freq, period=1e0):
        ashift, awidth = get_ashift_awidth(port)
        self.start       = Signal(reset=1)
        self.done        = Signal()
        self.base        = Signal(awidth)
        self.end         = Signal(awidth)
        self.length      = Signal(awidth)
        self.ticks = Signal(64)


        self.mydma_enables = CSRStorage(2, fields=[
                                   CSRField("mydma", description="_MyDMA", size=1),
                                   CSRField("litevideodma", description="DMAReader from litevideo", size=1),
                               ], description="Enable DMA",)




        # DMA --------------------------------------------------------------------------------------
        self.dma = dma = LiteDRAMDMAReader(port, fifo_buffered=False)
        self.submodules += dma

        # Address FSM ------------------------------------------------------------------------------
        cmd_counter = Signal(port.address_width, reset_less=True)

        #NOTE Right now we just dump 1kB of IQ data at 0x41000000 with boot.json.
        #TODO use a fsm, similar to _LiteDRAMBISTChecker or _LiteDRAMBISTGenerator to excercise the address and map it to the registers wired to the DAC?
        # I mean (dac_plat.data_a and dac_plat.data_b) instead of using the _MyDAC module
        #TODO Secondly this should output in an AXIStreamInterface

        #self.data_iq_addr = CSRStorage(32, description="Address of our IQ samples in memory", write_from_dev=True)
        self.data_iq_addr = Signal(32)
        self.data_iq = CSRStorage(32, fields=[
                                   CSRField("i", description="Data I", size=10),
                                   CSRField("q", description="Data Q", size=10),
                               ], description="IQ sample",)
        #data_a.eq(self.data_iq_storage.storage[0:10])
        #data_b.eq(self.data_iq_storage.storage[10:20])
        self.output_sig = Signal(dma.port.data_width)

        #[07:59:40] <_florent_> nickoe: LiteDRAMDMAReader has two endpoints: a sink to provide your read request and a source that will return the data
        #[08:00:20] <_florent_> so you can just set sink.valid.eq(1), sink.address.eq(the_address_you_want_to_read)
        #[08:00:31] <_florent_> then wait sink.ready to be 1
        #[08:00:57] <_florent_> and data will be returned on source.data when source.valid is 1


        if isinstance(port, LiteDRAMNativePort): # addressing in dwords
            dma_sink_addr = dma.sink.address
        else:
            raise NotImplementedError

        self.run_cascade_in  = Signal(reset=1)
        self.run_cascade_out = Signal()

        #start_addr = 0x41000000
        start_addr = int(0x1000000/(dma.port.data_width/8))
        #start_addr = 0x0400000

        # Data / Address FSM -----------------------------------------------------------------------
        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(self.start,
                NextValue(self.data_iq_addr, start_addr),
                NextState("RUN")
            ),
            NextValue(self.ticks, 0)
        )
        fsm.act("RUN",
            #dma.sink.valid.eq(1),
            dma.sink.valid.eq(self.mydma_enables.storage[0]),
            dma.source.ready.eq(1),
            If(dma.sink.ready,
                If(self.ticks[0] == 1, #hold address for two cycles, as it ix x16 SDRAM
                    NextValue(self.data_iq_addr, self.data_iq_addr + 1),
                ),
                If(self.data_iq_addr == (start_addr + 1024),
                    NextState("DONE")
                ),
               NextValue(self.ticks, self.ticks + 1),
            ),
            Display("ticks: %d\n", self.ticks),
        )
        fsm.act("DONE",
            self.done.eq(1)
        )

        self.comb += [
            dma_sink_addr.eq(self.data_iq_addr),
            self.output_sig.eq(dma.source.data)
        ]



# Simulation SoC -----------------------------------------------------------------------------------

class SimSoC(SoCCore):
    mem_map = {
        "ethmac": 0xb0000000,
    }
    mem_map.update(SoCCore.mem_map)

    def __init__(self,
        with_sdram            = False,
        with_ethernet         = False,
        with_etherbone        = False,
        etherbone_mac_address = 0x10e2d5000001,
        etherbone_ip_address  = "192.168.1.51",
        with_analyzer         = False,
        sdram_module          = "MT48LC16M16",
        sdram_init            = [],
        sdram_data_width      = 32,
        sdram_spd_data        = None,
        sdram_verbosity       = 0,
        with_sdcard           = False,
        sim_debug             = False,
        trace_reset_on        = False,
        trace_hack            = False,
        **kwargs):
        platform     = Platform()
        sys_clk_freq = int(1e6)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident         = "LiteX Simulation",
            ident_version = True,
            **kwargs)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = CRG(platform.request("sys_clk"))

        # SDRAM ------------------------------------------------------------------------------------
        if with_sdram:
            sdram_clk_freq = int(100e6) # FIXME: use 100MHz timings
            if sdram_spd_data is None:
                sdram_module_cls = getattr(litedram_modules, sdram_module)
                sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
                sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
            else:
                sdram_module = litedram_modules.SDRAMModule.from_spd_data(sdram_spd_data, sdram_clk_freq)
            phy_settings     = get_sdram_phy_settings(
                memtype    = sdram_module.memtype,
                data_width = sdram_data_width,
                clk_freq   = sdram_clk_freq)
            self.submodules.sdrphy = SDRAMPHYModel(
                module    = sdram_module,
                settings  = phy_settings,
                clk_freq  = sdram_clk_freq,
                verbosity = sdram_verbosity,
                init      = sdram_init)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = sdram_module,
                origin                  = self.mem_map["main_ram"],
                size                    = kwargs.get("max_sdram_size", 0x40000000),
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = False
            )
            # Reduce memtest size for simulation speedup
            self.add_constant("MEMTEST_DATA_SIZE", 8*1024)
            self.add_constant("MEMTEST_ADDR_SIZE", 8*1024)
            self.add_constant("SDRAM_TEST_DISABLE")  # Skip SDRAM test to avoid corrupting pre-initialized contents.

        #assert not (with_ethernet and with_etherbone)

        if with_ethernet and with_etherbone:
            etherbone_ip_address = convert_ip(etherbone_ip_address)
            # Ethernet PHY
            self.submodules.ethphy = LiteEthPHYModel(self.platform.request("eth", 0))
            self.add_csr("ethphy")
            # Ethernet MAC
            self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=8,
                interface  = "hybrid",
                endianness = self.cpu.endianness,
                hw_mac     = etherbone_mac_address)

            # SoftCPU
            self.add_memory_region("ethmac", self.mem_map["ethmac"], 0x2000, type="io")
            self.add_wb_slave(self.mem_regions["ethmac"].origin, self.ethmac.bus, 0x2000)
            self.add_csr("ethmac")
            if self.irq.enabled:
                self.irq.add("ethmac", use_loc_if_exists=True)
            # HW ethernet
            self.submodules.arp  = LiteEthARP(self.ethmac, etherbone_mac_address, etherbone_ip_address, sys_clk_freq, dw=8)
            self.submodules.ip   = LiteEthIP(self.ethmac, etherbone_mac_address, etherbone_ip_address, self.arp.table, dw=8)
            self.submodules.icmp = LiteEthICMP(self.ip, etherbone_ip_address, dw=8)
            self.submodules.udp  = LiteEthUDP(self.ip, etherbone_ip_address, dw=8)
            # Etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.udp, 1234, mode="master")
            self.add_wb_master(self.etherbone.wishbone.bus)

        # Ethernet ---------------------------------------------------------------------------------
        elif with_ethernet:
            # Ethernet PHY
            self.submodules.ethphy = LiteEthPHYModel(self.platform.request("eth", 0))
            self.add_csr("ethphy")
            # Ethernet MAC
            ethmac = LiteEthMAC(phy=self.ethphy, dw=32,
                interface  = "wishbone",
                endianness = self.cpu.endianness)
            if with_etherbone:
                ethmac = ClockDomainsRenamer({"eth_tx": "ethphy_eth_tx", "eth_rx":  "ethphy_eth_rx"})(ethmac)
            self.submodules.ethmac = ethmac
            self.add_memory_region("ethmac", self.mem_map["ethmac"], 0x2000, type="io")
            self.add_wb_slave(self.mem_regions["ethmac"].origin, self.ethmac.bus, 0x2000)
            self.add_csr("ethmac")
            if self.irq.enabled:
                self.irq.add("ethmac", use_loc_if_exists=True)

        # Etherbone --------------------------------------------------------------------------------
        elif with_etherbone:
            # Ethernet PHY
            self.submodules.ethphy = LiteEthPHYModel(self.platform.request("eth", 0)) # FIXME
            self.add_csr("ethphy")
            # Ethernet Core
            ethcore = LiteEthUDPIPCore(self.ethphy,
                mac_address = etherbone_mac_address,
                ip_address  = etherbone_ip_address,
                clk_freq    = sys_clk_freq)
            self.submodules.ethcore = ethcore
            # Etherbone
            self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234, mode="master")
            self.add_wb_master(self.etherbone.wishbone.bus)

        # Leds -------------------------------------------------------------------------------------
        self.submodules.leds = LedChaser(
            pads         = platform.request_all("user_led"),
            sys_clk_freq = sys_clk_freq)
        self.add_csr("leds")

        # Attempt to create DMA to AXI Stream ------------------------------------------------------
        # Just connecting it directly to the DAC pins, 2 x 10 bit
        # RAM ->  _MyDMA -> AXI Stream -> DAC pins
        # LiteDRAMDMAReader -> AXIStreamInterface -> dac_plat?

        dac_plat = platform.request("dac")
        '''
        self.submodules.dac = _MyDAC(
            data_a=dac_plat.data_a,
            data_b=dac_plat.data_b,
            cw=dac_plat.cw,
            sys_clk_freq=sys_clk_freq)
        '''
        self.add_csr("dac")


        # Create our platform (fpga interface)
        platform.add_source("blink.v")
        # Create our module (fpga description)
        module = Module()
        module.specials += Instance("blink",
                                    i_clk=ClockSignal(),
                                    i_rst=ResetSignal(),
                                    o_led=None
                                    )
        self.submodules.blink = module





        #self.submodules.mydma = medma = _MyDMA(self.sdram.crossbar.get_port(mode="read"), sys_clk_freq)
        self.submodules.mydma = medma = _MyDMA(self.sdram.crossbar.get_port(mode="read", data_width=32), sys_clk_freq)
        self.add_csr("mydma")


        '''
        self.submodules.dma2 = dma = DMAReader(self.sdram.crossbar.get_port(mode="read",  data_width=8),fifo_depth=4)
        self.output_sig2 = Signal(8)
        self.comb += [
            dma.sink.valid.eq(medma.mydma_enables.storage[1]),
            dma.source.ready.eq(1),
            self.output_sig2.eq(dma.source.data)
        ]
        self.add_csr("dma2")
        '''

        self.dac_sig_data = Signal(32)
        self.dac_sig_ncw = Signal(1)
        self.ticks = Signal(4)
        self.dac_converted_done = Signal(1)

        self.comb += [
            self.dac_sig_data.eq(medma.dma.source.data),
            self.dac_sig_ncw.eq(medma.dma.source.valid)
        ]



        # Create our platform (fpga interface)
        platform.add_source("dac.v")
        # Create our module (fpga description)
        dac_vmodule = Module()


        # TODO connect up the i_t<somthing> signals to the AXI stream
        module.specials += Instance("dac",
                                    i_i_clk=ClockSignal(),
                                    i_i_reset=ResetSignal(),
                                    i_i_tdata=Cat(medma.dma.source.data[0:10], medma.dma.source.data[16:26] ),
                                    i_i_tvalid=medma.dma.source.valid,
                                    o_o_tready=None,
                                    o_o_sig_a=dac_plat.data_a,
                                    o_o_sig_b=dac_plat.data_b,
                                    o_o_ncw=dac_plat.cw
                                    )
        self.submodules.dac_module = dac_vmodule


        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                # IBus (could also just added as self.cpu.ibus)
                #self.cpu.ibus.stb,
                #self.cpu.ibus.cyc,
                # self.cpu.ibus.adr,
                # self.cpu.ibus.we,
                # self.cpu.ibus.ack,
                # self.cpu.ibus.sel,
                # self.cpu.ibus.dat_w,
                # self.cpu.ibus.dat_r,
                # # DBus (could also just added as self.cpu.dbus)
                # self.cpu.dbus.stb,
                # self.cpu.dbus.cyc,
                # self.cpu.dbus.adr,
                # self.cpu.dbus.we,
                # self.cpu.dbus.ack,
                # self.cpu.dbus.sel,
                # self.cpu.dbus.dat_w,
                # self.cpu.dbus.dat_r,
                platform.lookup_request("user_led", 0),
                platform.lookup_request("user_led", 1),
                platform.lookup_request("user_led", 2),
                platform.lookup_request("user_led", 3),
                #self.dac,
                #self.sdrphy,
                #self.user_leds,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 512,
                clock_domain = "sys",
                csr_csv      = "analyzer.csv")
            self.add_csr("analyzer")

        # SDCard -----------------------------------------------------------------------------------
        if with_sdcard:
            self.add_sdcard("sdcard", use_emulator=True)

        # Simulation debugging ----------------------------------------------------------------------
        if sim_debug:
            platform.add_debug(self, reset=1 if trace_reset_on else 0)
        else:
            #self.comb += platform.trace.eq(1)
            self.comb += platform.trace.eq(medma.mydma_enables.storage[1])


        if trace_hack:
            #cycles_end = 2000000
            cycles_end = 20000
            cycles = Signal(32)
            self.sync += If( (medma.mydma_enables.storage[1] == 1),
            cycles.eq(cycles + 1)
            )
            #self.sync += If(cycles == cycles_end, Finish())

            self.sync += If(cycles == cycles_end,
                Display("-"*80),
                Display("Cycles: %d", cycles),
                #Display("Errors: %d", errors),
                Display("-"*80),
                Finish(),
            )


# Build --------------------------------------------------------------------------------------------

def generate_gtkw_savefile(builder, vns, trace_fst):
    from litex.build.sim import gtkwave as gtkw
    dumpfile = os.path.join(builder.gateware_dir, "sim.{}".format("fst" if trace_fst else "vcd"))
    savefile = os.path.join(builder.gateware_dir, "sim.gtkw")
    soc = builder.soc

    with gtkw.GTKWSave(vns, savefile=savefile, dumpfile=dumpfile) as save:
        save.clocks()
        save.fsm_states(soc)
        save.add(soc.bus.slaves["main_ram"], mappers=[gtkw.wishbone_sorter(), gtkw.wishbone_colorer()])

        if hasattr(soc, 'sdrphy'):
            # all dfi signals
            save.add(soc.sdrphy.dfi, mappers=[gtkw.dfi_sorter(), gtkw.dfi_in_phase_colorer()])

            # each phase in separate group
            with save.gtkw.group("dfi phaseX", closed=True):
                for i, phase in enumerate(soc.sdrphy.dfi.phases):
                    save.add(phase, group_name="dfi p{}".format(i), mappers=[
                        gtkw.dfi_sorter(phases=False),
                        gtkw.dfi_in_phase_colorer(),
                    ])

            # only dfi command/data signals
            def dfi_group(name, suffixes):
                save.add(soc.sdrphy.dfi, group_name=name, mappers=[
                    gtkw.regex_filter(gtkw.suffixes2re(suffixes)),
                    gtkw.dfi_sorter(),
                    gtkw.dfi_per_phase_colorer(),
                ])

            dfi_group("dfi commands", ["cas_n", "ras_n", "we_n"])
            dfi_group("dfi commands", ["wrdata"])
            dfi_group("dfi commands", ["wrdata_mask"])
            dfi_group("dfi commands", ["rddata"])

def sim_args(parser):
    builder_args(parser)
    soc_sdram_args(parser)
    parser.add_argument("--threads",              default=1,               help="Set number of threads (default=1)")
    parser.add_argument("--rom-init",             default=None,            help="rom_init file")
    parser.add_argument("--ram-init",             default=None,            help="ram_init file")
    parser.add_argument("--with-sdram",           action="store_true",     help="Enable SDRAM support")
    parser.add_argument("--sdram-module",         default="MT48LC16M16",   help="Select SDRAM chip")
    parser.add_argument("--sdram-data-width",     default=32,              help="Set SDRAM chip data width")
    parser.add_argument("--sdram-init",           default=None,            help="SDRAM init file")
    parser.add_argument("--sdram-from-spd-dump",  default=None,            help="Generate SDRAM module based on data from SPD EEPROM dump")
    parser.add_argument("--sdram-verbosity",      default=0,               help="Set SDRAM checker verbosity")
    parser.add_argument("--with-ethernet",        action="store_true",     help="Enable Ethernet support")
    parser.add_argument("--with-etherbone",       action="store_true",     help="Enable Etherbone support")
    parser.add_argument("--local-ip",             default="192.168.1.50",  help="Local IP address of SoC (default=192.168.1.50)")
    parser.add_argument("--remote-ip",            default="192.168.1.100", help="Remote IP address of TFTP server (default=192.168.1.100)")
    parser.add_argument("--with-analyzer",        action="store_true",     help="Enable Analyzer support")
    parser.add_argument("--with-sdcard",          action="store_true",     help="Enable SDCard support")
    parser.add_argument("--trace",                action="store_true",     help="Enable Tracing")
    parser.add_argument("--trace-fst",            action="store_true",     help="Enable FST tracing (default=VCD)")
    parser.add_argument("--trace-start",          default="0",             help="Time to start tracing (ps)")
    parser.add_argument("--trace-end",            default="-1",            help="Time to end tracing (ps)")
    parser.add_argument("--opt-level",            default="O3",            help="Compilation optimization level")
    parser.add_argument("--sim-debug",            action="store_true",     help="Add simulation debugging modules")
    parser.add_argument("--gtkwave-savefile",     action="store_true",     help="Generate GTKWave savefile")

def main():
    parser = argparse.ArgumentParser(description="Generic LiteX SoC Simulation")
    sim_args(parser)
    args = parser.parse_args()

    soc_kwargs     = soc_sdram_argdict(args)
    builder_kwargs = builder_argdict(args)

    sys_clk_freq = int(1e6)
    sim_config = SimConfig()
    sim_config.add_clocker("sys_clk", freq_hz=sys_clk_freq)

    # Configuration --------------------------------------------------------------------------------

    cpu = CPUS[soc_kwargs.get("cpu_type", "vexriscv")]
    if soc_kwargs["uart_name"] == "serial":
        soc_kwargs["uart_name"] = "sim"
        sim_config.add_module("serial2console", "serial")
    if args.rom_init:
        soc_kwargs["integrated_rom_init"] = get_mem_data(args.rom_init, cpu.endianness)
    if not args.with_sdram:
        soc_kwargs["integrated_main_ram_size"] = 0x10000000 # 256 MB
        if args.ram_init is not None:
            soc_kwargs["integrated_main_ram_init"] = get_mem_data(args.ram_init, cpu.endianness)
    else:
        assert args.ram_init is None
        soc_kwargs["integrated_main_ram_size"] = 0x0
        soc_kwargs["sdram_module"]             = args.sdram_module
        soc_kwargs["sdram_data_width"]         = int(args.sdram_data_width)
        soc_kwargs["sdram_verbosity"]          = int(args.sdram_verbosity)
        if args.sdram_from_spd_dump:
            soc_kwargs["sdram_spd_data"] = parse_spd_hexdump(args.sdram_from_spd_dump)

    if args.with_ethernet or args.with_etherbone:
        sim_config.add_module("ethernet", "eth", args={"interface": "tap0", "ip": args.remote_ip})

    trace_start = int(float(args.trace_start))
    trace_end = int(float(args.trace_end))


    '''
    ram_init = []
    ram_init = get_mem_data({
        #"./build/sim/software/bios/bios.bin": "0x00000000",
        #"test_data.cs16": "0x41000000",
        "demo.bin": "0x40000000",
    }, cpu.endianness)
    '''



    # SoC ------------------------------------------------------------------------------------------
    soc = SimSoC(
        with_sdram     = args.with_sdram,
        with_ethernet  = args.with_ethernet,
        with_etherbone = args.with_etherbone,
        with_analyzer  = args.with_analyzer,
        with_sdcard    = args.with_sdcard,
        sim_debug      = args.sim_debug,
        trace_reset_on = trace_start > 0 or trace_end > 0,
        trace_hack     = args.trace,
        sdram_init     = [] if args.sdram_init is None else get_mem_data(args.sdram_init, cpu.endianness),
        #sdram_init     = [] if args.sdram_init is None else ram_init,
        **soc_kwargs)
    if args.ram_init is not None or args.sdram_init is not None:
        soc.add_constant("ROM_BOOT_ADDRESS", 0x40000000)
    if args.with_ethernet:
        for i in range(4):
            soc.add_constant("LOCALIP{}".format(i+1), int(args.local_ip.split(".")[i]))
        for i in range(4):
            soc.add_constant("REMOTEIP{}".format(i+1), int(args.remote_ip.split(".")[i]))

    # Build/Run ------------------------------------------------------------------------------------
    builder_kwargs["csr_csv"] = "csr.csv"
    builder = Builder(soc, **builder_kwargs)
    for i in range(2):
        build = (i == 0)
        run   = (i == 1)
        vns = builder.build(
            build       = build,
            run         = run,
            threads     = args.threads,
            sim_config  = sim_config,
            opt_level   = args.opt_level,
            trace       = args.trace,
            trace_fst   = args.trace_fst,
            trace_start = trace_start,
            trace_end   = trace_end
        )
        if args.with_analyzer:
            soc.analyzer.export_csv(vns, "analyzer.csv")
        if args.gtkwave_savefile:
            generate_gtkw_savefile(builder, vns, args.trace_fst)

if __name__ == "__main__":
    main()
