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
    def __init__(self, platform, pads, dac_clk):
        self.sink = sink = Endpoint([('data', 32)])

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
                                    i_i_tvalid=None,
                                    o_o_tready=None,
                                    o_o_sig_a=pads.data_a,
                                    o_o_sig_b=pads.data_b,
                                    o_o_ncw=pads.cw
                                    )
        self.specials += dac_vmodule


class MyDMA(Module, AutoCSR):
    def __init__(self, port, clock_domain, period=1e0):
        ashift, awidth = get_ashift_awidth(port)
        self.start       = Signal(reset=1)
        self.done        = Signal()
        self.base        = Signal(awidth)
        self.end         = Signal(awidth)
        self.length      = Signal(awidth)
        self.ticks = Signal(16)


        self.mydma_enables = CSRStorage(2, fields=[
                                   CSRField("mydma", description="_MyDMA", size=1),
                                   CSRField("litevideodma", description="DMAReader from litevideo", size=1),
                               ], description="Enable DMA",)



        depth = 16
        # DMA --------------------------------------------------------------------------------------
        self.dma = dma = LiteDRAMDMAReader(port, fifo_depth=depth, fifo_buffered=False)
        self.submodules += dma

        # Adding a FIFO that can work between clock domains to optimize memory access
        self.myfifo = myfifo = stream.AsyncFIFO([("data", dma.port.data_width)], depth * 2)
        self.submodules += ClockDomainsRenamer({"write": "sys", "read": "dac"})(myfifo)
        self.submodules += myfifo

        self.submodules.cdc = cdc = stream.ClockDomainCrossing([("data", dma.port.data_width)], cd_from="sys", cd_to=clock_domain)
        self.comb += self.dma.source.connect(self.cdc.sink)

        #TODO how to attack to myfifo? Another connect?
        #self.comb += cdc.source.connect_flat()

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



        #start_addr = 0x41000000
        start_addr = int(0x1000000/(dma.port.data_width/8))
        #start_addr = 0x0400000
        self.Q = Signal(1)
        self.Qi = Signal(1)
        #self.sync += self.Q.eq(dac_clk)
        self.comb += self.Qi.eq(~self.Q)

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
                # dma.sink.valid.eq(1),
                dma.sink.valid.eq(self.mydma_enables.storage[0]),
                dma.source.ready.eq(1),
                If(dma.sink.ready,
                   If(self.ticks[0] == 1,  # hold address for two cycles, as it ix x16 SDRAM
                      NextValue(self.data_iq_addr, self.data_iq_addr + 1),
                      ),
                   If(self.data_iq_addr == (start_addr + 1024),
                      NextState("DONE")
                      ),
                   NextValue(self.ticks, self.ticks + 1),
                   ),
                )
        fsm.act("DONE",
                self.done.eq(1)
                )

        self.comb += [
            dma_sink_addr.eq(self.data_iq_addr),
            #self.output_sig.eq(dma.source.data)
        ]
