# Custom modules and stuff that can be shared between the simulation and implementation scritps, the goal being to keep
# the root scripts a bit cleaner.

from migen import *
from litex.soc.integration.soc import *
from litex.build.generic_platform import *
from litex.build.sim import SimPlatform


from litedram.frontend.bist import get_ashift_awidth
from litedram.frontend.dma import LiteDRAMDMAWriter, LiteDRAMDMAReader
from litedram.common import LiteDRAMNativePort

from litex.soc.interconnect import stream
from litex.soc.interconnect.stream import Endpoint

from litevideo.output.core import DMAReader




# IOs for simulation -------------------------------------------------------------------------------

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

class MySimPlatform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)


# Other stuff
class AlexandersDAC(Module):
    #MAX5854 DAC
    def __init__(self, platform, dac_clk):
        self.sink = sink = Endpoint([('data', 32)])

        pads = platform.request("dac")

        # We just takes the 10 bit from each byte pair needed
        intermediate_signal = Signal(20)
        self.comb += intermediate_signal.eq(Cat(sink.data[0:10], self.sink.data[16:26]))

        # Create our platform (fpga interface)
        platform.add_source("dac.v")
        # Create our module (fpga description)
        dac_vmodule = Instance("dac",
                                    i_i_clk=dac_clk.clk,
                                    i_i_reset=ResetSignal(),
                                    i_i_tdata=intermediate_signal,
                                    i_i_tvalid=sink.valid,
                                    o_o_tready=sink.ready,
                                    o_o_sig_a=pads.data_a,
                                    o_o_sig_b=pads.data_b,
                                    o_o_ncw=pads.cw
                                    )
        self.specials += dac_vmodule


        # clocking for the DAC
        # As the differential output is in the same bank as the other signals that are LVCMOS33 we
        # can't really drive this as a diff out. :(
        # And the logic inputs needs 0.64*DVdd = 2.145 V for high state..
        #self.specials += DifferentialOutput(self.crg.cd_dac.clk, dac_plat.clkx_p, dac_plat.clkx_n)

        self.comb += pads.clkx_p.eq(dac_clk.clk)
        #self.comb += pads.clkx_n.eq(self.crg.cd_dac_180.clk)
        self.comb += pads.clkx_test.eq(dac_clk.clk)
        #self.comb += pads.clkx_test2.eq(self.crg.cd_dac_180.clk)



class MyDMA(Module, AutoCSR):
    def __init__(self,platform, port, clock_domain, period=1e0):
        ashift, awidth = get_ashift_awidth(port)
        self.start       = Signal(reset=1)
        self.done        = Signal()
        self.base        = Signal(awidth)
        self.end         = Signal(awidth)
        self.length      = Signal(awidth)
        self.ticks = Signal(32)
        self.ticks_tic = Signal(32)


        self.mydma_enables = CSRStorage(2, fields=[
                                   CSRField("mydma", description="_MyDMA", size=1),
                                   CSRField("litevideodma", description="DMAReader from litevideo", size=1),
                               ], description="Enable DMA",)




        # DMA --------------------------------------------------------------------------------------
        own_dma = 0
        if own_dma:
            self.dma = dma = LiteDRAMDMAReader(port, fifo_depth=16, fifo_buffered=False)
            self.submodules += dma
        else:
            self.submodules.dma = dma = DMAReader(port, fifo_depth=64)
            self.output_sig2 = Signal(32)
            self.comb += [
                dma.sink.valid.eq(self.mydma_enables.storage[1]),
                dma.source.ready.eq(1),
                dma.sink.base.eq(0x41000000),
                dma.sink.length.eq(1024),
                self.output_sig2.eq(dma.source.data)
            ]

        self.submodules.cdc = cdc = stream.ClockDomainCrossing([("data", port.data_width)], cd_from="sys", cd_to=clock_domain.name, depth=32)
        self.comb += self.dma.source.connect(self.cdc.sink)

        self.submodules.dac = dac = AlexandersDAC(platform, clock_domain)
        self.comb += cdc.source.connect(dac.sink)

        #NOTE Right now we just dump 1kB of IQ data at 0x41000000 with boot.json.

        #self.data_iq_addr = CSRStorage(32, description="Address of our IQ samples in memory", write_from_dev=True)
        self.data_iq_addr = Signal(32)
        self.data_iq = CSRStorage(32, fields=[
                                   CSRField("i", description="Data I", size=10),
                                   CSRField("q", description="Data Q", size=10),
                               ], description="IQ sample",)
        #data_a.eq(self.data_iq_storage.storage[0:10])
        #data_b.eq(self.data_iq_storage.storage[10:20])
        self.output_sig = Signal(port.data_width)

        if own_dma:
            if isinstance(port, LiteDRAMNativePort): # addressing in dwords
                dma_sink_addr = dma.sink.address
            else:
                raise NotImplementedError

            #start_addr = 0x41000000
            start_addr = int(0x1000000/(port.data_width/8))
            #start_addr = 0x0400000

            fsm = FSM(reset_state="IDLE")
            self.submodules += fsm
            fsm.act("IDLE",
                    If(self.start,
                       NextValue(self.data_iq_addr, start_addr),
                       NextState("RUN")
                       ),
                    NextValue(self.ticks, 0),
                    NextValue(self.ticks_tic, 0),
                    )
            fsm.act("RUN",
                    dma.sink.valid.eq(self.mydma_enables.storage[0]),
                    NextValue(self.ticks, self.ticks + 1),
                    If(self.data_iq_addr == (start_addr + 1024),
                       NextState("DONE")
                       ),
                    #If(dma.sink.ready & (self.ticks_tic+4 < self.ticks),
                    If(dma.sink.ready,
                       NextValue(self.data_iq_addr, self.data_iq_addr + 1),
                       NextValue(self.ticks_tic, self.ticks)
                       ),
                    )
            fsm.act("DONE",
                    self.done.eq(1),
                    NextState("IDLE"),
                    )

            self.comb += [
                dma_sink_addr.eq(self.data_iq_addr),
            ]
