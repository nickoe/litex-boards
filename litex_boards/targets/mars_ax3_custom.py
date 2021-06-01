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
from litex.soc.interconnect.stream import Endpoint, Pipeline

from litevideo.output.core import DMAReader

import struct



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
class NicksDAC(Module):
    def __init__(self, platform, dac_clk, dac_clk2):
        self.sink = sink = Endpoint([('data', 32)])
        self.running     = Signal(1)
        self.ticks       = Signal(32)


        pads = platform.request("dac")

        # We just takes the 10 bit from each byte pair needed
        intermediate_signal = Signal(20)
        self.comb += intermediate_signal.eq(Cat(sink.data[0:10], self.sink.data[16:26]))

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
                If(~self.running,
                   NextState("CONFIG")
                   ),
                NextValue(self.ticks, 0),
                )
        fsm.act("CONFIG",
                NextValue(self.ticks, self.ticks + 1),
                NextValue(pads.data_a, 0x120),
                NextValue(pads.data_b, 0),
                NextValue(pads.cw, 0),
                NextValue(sink.ready, 1),
                NextState("RUN"),
                )
        fsm.act("RUN",
                If(sink.valid,
                    NextValue(self.ticks, self.ticks + 1),
                    NextValue(pads.data_a, sink.data[0:10]),
                    NextValue(pads.data_b, sink.data[16:26]),
                    NextValue(pads.cw, 1),
                    NextValue(self.running, 1),
                ).Else(
                    NextValue(self.running, 0),
                    NextValue(sink.ready, 0),
                    NextState("IDLE")
                    ),
                )

        # clocking for the DAC
        # As the differential output is in the same bank as the other signals that are LVCMOS33 we
        # can't really drive this as a diff out. :(
        # And the logic inputs needs 0.64*DVdd = 2.145 V for high state..
        #self.specials += DifferentialOutput(self.crg.cd_dac.clk, dac_plat.clkx_p, dac_plat.clkx_n)
        self.comb += pads.clkx_p.eq(dac_clk.clk)
        self.comb += pads.clkx_n.eq(dac_clk2.clk)
        self.comb += pads.clkx_test.eq(dac_clk.clk)
        self.comb += pads.clkx_test2.eq(dac_clk2.clk)


class AlexandersDAC(Module):
    #MAX5854 DAC
    def __init__(self, platform, dac_clk, dac_clk2):
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
        self.comb += pads.clkx_n.eq(dac_clk2.clk)
        self.comb += pads.clkx_test.eq(dac_clk.clk)
        self.comb += pads.clkx_test2.eq(dac_clk2.clk)

class NullSink(Module):
    def __init__(self, platform, width=10):
        self.sink = sink = Endpoint([('data', width)])
        self.comb += sink.ready.eq(1)


class BbFilter(Module):
    '''
    Upsampler and RRcos filter (baseband filter)
    '''

    def __init__(self, platform, clk_domain: ClockDomain):
        iw =width = 16  # filter bus width
        self.sink = sink = Endpoint([('data', iw)])
        self.source = source = Endpoint([('data', iw)])
        ntaps = 256
        # taps = Array(list(1 for i in range(256)))
        mem_size = 2 ** 8
        # taps_data = list([7, 14, 21])
        # taps_data = list(range(ntaps))
        # taps_data = [7,3,5]
        # taps_data = "filtercoeffs_16.dat"
        taps_data = []
        f = open("filtercoeffs_16.dat", "r")
        for line in f:
            a = line[0:4]
            # b = struct.unpack('>h', bytes.fromhex(a))
            # c = b[0]
            taps_data.append(int(a, 16))
        #ntaps = len(taps_data)
        self.specials.mem_taps = mem_taps = Memory(32, mem_size, init=taps_data)

        offset_counter = Signal(max=mem_size)
        # oc_load = Signal()
        oc_inc = Signal()
        self.sync += \
            If(oc_inc,
               offset_counter.eq(offset_counter + 1)
               )

        rdport = mem_taps.get_port()
        self.specials += rdport
        self.comb += rdport.adr.eq(offset_counter)

        self.filter_out = Signal(iw)
        self.load = Signal(1)
        self.comb += source.data.eq(self.filter_out)

        self.tap_sig = Signal(32)
        self.ticks = Signal(32)

        self.upsample_num = CSRStorage(8, reset=16)


        # Create our platform (fpga interface)
        platform.add_source("bb_filter/BbFilterAXI.v")
        platform.add_source("rrcos/firtap.v")
        platform.add_source("rrcos/genericfir.v")
        platform.add_source("rrcos/rrcosAXI.v")
        platform.add_source("upsampler/upsampleAXI.v")

        self.enable_filter = Signal(1)
        self.local_reset = Signal(1)

        # Create our module (fpga description)
        rrcosfilter_vmodule = Instance("BbFilterAXI",
                                       i_i_clk=clk_domain.clk,
                                       #i_i_reset=ResetSignal(),
                                       i_i_reset=self.local_reset,
                                       i_i_data=sink.data[0:iw],
                                       o_o_data=self.filter_out,
                                       # i_i_taps=taps,
                                       i_i_taps=rdport.dat_r[0:iw],
                                       i_i_L=self.upsample_num.storage,
                                       i_i_load=self.load,
                                       #i_i_tvalid=sink.valid,
                                       i_i_tvalid=self.enable_filter,
                                       o_i_tready=sink.ready,
                                       i_o_tvalid=source.valid,
                                       i_o_tready=source.ready,
                                       # p_IW=iw,
                                       )
        self.specials += rrcosfilter_vmodule

        fsm_fisk = FSM(reset_state="INIT")
        self.submodules += fsm_fisk
        fsm_fisk.act("INIT",
                NextValue(self.ticks, 0),
                self.load.eq(0),
                self.local_reset.eq(1),
                NextValue(offset_counter, 0),
                If(sink.valid,
                   self.enable_filter.eq(0),
                   #NextState("LOAD_TAPS"),
                   NextState("ZERO_TAPS"),
                   )
                )
        fsm_fisk.act("ZERO_TAPS",
                NextValue(self.ticks, self.ticks + 1),
                NextValue(self.load, 1),
                If((ntaps - len(taps_data) - 1) == self.ticks,
                   NextState("LOAD_TAPS"),
                   )
                )
        fsm_fisk.act("LOAD_TAPS",
                NextValue(self.ticks, self.ticks + 1),
                # NextValue(self.tap_sig, mem_taps[self.ticks]),
                # NextValue(oc_inc, 1),
                oc_inc.eq(1),
                # sink.ready(1), # don't harcode? attack to
                NextValue(self.load, 1),
                If(ntaps   - 1 == self.ticks,
                   # NextValue(oc_inc, 0),
                   oc_inc.eq(0),
                   self.load.eq(0),
                   NextState("RUN"),
                   )
                )
        fsm_fisk.act("RUN",
                NextValue(self.ticks, self.ticks + 1),
                self.enable_filter.eq(sink.valid),
                # If(self.ticks > 1024,
                # Fake wait until retry
                NextValue(self.ticks, 0x10FEDABE),
                #   NextState("INIT"),
                #   )

                )



class RRcosFilter(Module):
    '''
    RRcos filter
    '''
    def __init__(self, platform, clk):


        iw=16 # filter bus width
        self.sink = sink = Endpoint([('data', iw)])
        self.source = source = Endpoint([('data', iw)])
        #ntaps = 256
        #taps = Array(list(1 for i in range(256)))
        mem_size = 2**8
        #taps_data = list([7, 14, 21])
        #taps_data = list(range(ntaps))
        #taps_data = [7,3,5]
        #taps_data = "filtercoeffs_16.dat"
        taps_data = []
        f = open("filtercoeffs_16.dat", "r")
        for line in f:
            a = line[0:4]
            #b = struct.unpack('>h', bytes.fromhex(a))
            #c = b[0]
            taps_data.append(int(a,16))
        ntaps = len(taps_data)
        self.specials.mem_taps = mem_taps = Memory(32, mem_size, init=taps_data)

        offset_counter = Signal(max=mem_size)
        #oc_load = Signal()
        oc_inc = Signal()
        self.sync += \
            If(oc_inc,
                offset_counter.eq(offset_counter + 1)
            )

        rdport = mem_taps.get_port()
        self.specials += rdport
        self.comb += rdport.adr.eq(offset_counter)



        self.filter_out = Signal(iw)
        self.load = Signal(1)
        self.comb += source.data.eq(self.filter_out)

        self.tap_sig = Signal(32)
        self.ticks = Signal(32)


        # Create our platform (fpga interface)
        platform.add_source("rrcos/rrcosAXI.v")
        platform.add_source("rrcos/genericfir.v")
        platform.add_source("rrcos/firtap.v")
        # Create our module (fpga description)
        rrcosfilter_vmodule = Instance("rrcosAXI",
                                    i_i_clk=clk,
                                    i_i_reset=ResetSignal(),
                                    i_i_data=sink.data[0:iw],
                                    o_o_data=self.filter_out,
                                    #i_i_taps=taps,
                                    i_i_taps=rdport.dat_r[0:iw],
                                    i_i_load=self.load,
                                    i_i_tvalid=sink.valid,
                                    o_i_tready=sink.ready,
                                    i_o_tvalid=source.valid,
                                    i_o_tready=source.ready,
                                    #p_IW=iw,
                                    )
        self.specials += rrcosfilter_vmodule


        fsm = FSM(reset_state="INIT")
        self.submodules += fsm
        fsm.act("INIT",
                NextValue(self.ticks, 0),
                self.load.eq(0),
                NextValue(offset_counter, 0),
                If(sink.valid,
                #If(,
                    NextState("LOAD_TAPS"),
                    )
                )
        fsm.act("LOAD_TAPS",
                NextValue(self.ticks, self.ticks + 1),
                #NextValue(self.tap_sig, mem_taps[self.ticks]),
                #NextValue(oc_inc, 1),
                oc_inc.eq(1),
                #sink.ready(1), # don't harcode? attack to
                NextValue(self.load, 1),
                If(ntaps-1 == self.ticks,
                   #NextValue(oc_inc, 0),
                   oc_inc.eq(0),
                   self.load.eq(0),
                   NextState("RUN"),
                   )
                )
        fsm.act("RUN",
                NextValue(self.ticks, self.ticks + 1),
                #If(self.ticks > 1024,
                   # Fake wait until retry
                   NextValue(self.ticks, 0x10FEDABE),
                #   NextState("INIT"),
                #   )

                )


class Upsampler(Module, AutoCSR):
    '''
    Upsampler
    '''
    def __init__(self, platform, clk):


        width=16# filter bus width
        ntaps = 256

        self.sink = sink = Endpoint([('data', width)])
        self.source = source = Endpoint([('data', width)])

        self.upsample_num = CSRStorage(8, reset=16)

        self.filter_out = Signal(width)
        self.load = Signal(1)
        self.comb += source.data.eq(self.filter_out)

        # Create our platform (fpga interface)
        platform.add_source("upsampler/upsampleAXI.v")
        # Create our module (fpga description)
        upsampler_vmodule = Instance("upsampleAXI",
                                    i_i_clk=clk,
                                    i_i_reset=ResetSignal(),
                                    i_i_data=sink.data[0:width],
                                    o_o_data=self.filter_out,
                                    i_i_L=self.upsample_num.storage,
                                    i_i_load=self.load,
                                    i_i_tvalid=sink.valid,
                                    o_i_tready=sink.ready,
                                    i_o_tvalid=source.valid,
                                    i_o_tready=source.ready,
                                    p_DW=width,
                                    )

        self.specials += upsampler_vmodule


        self.ticks = Signal(32)
        fsm = FSM(reset_state="RESET")
        self.submodules += fsm
        fsm.act("RESET",
                NextValue(self.load, 0),
                NextValue(self.ticks, 0),
                NextState("CONFIG"),
                )
        fsm.act("CONFIG",
                If(self.ticks > 10,
                   NextState("RUN"),
                   NextValue(self.load, 1),
                   ),
                NextValue(self.ticks, self.ticks + 1),
                )
        fsm.act("RUN",
                #If(self.upsample_num.re,
                If(0,
                   NextState("RESET"),
                ).Else(
                   NextValue(self.ticks, self.ticks + 2),
                   NextValue(self.load, 0),
                   ),
                )


class MyDMA(Module, AutoCSR):
    def __init__(self,platform, port, clock_domain, clock_domain2):
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

        self.output_sig2 = Signal(32)

        # DMA --------------------------------------------------------------------------------------
        own_dma = 1
        if own_dma == 1:
            self.dma = dma = LiteDRAMDMAReader(port, fifo_depth=32, fifo_buffered=True, with_csr=True)
            self.submodules += dma
        else:
            self.submodules.dma = dma = DMAReader(port, fifo_depth=32)
            self.comb += [
                dma.sink.valid.eq(self.mydma_enables.storage[1]),
                dma.source.ready.eq(1),
                dma.sink.base.eq(0x41000000),
                dma.sink.length.eq(1024),
                self.output_sig2.eq(dma.source.data)
            ]

        #self.submodules.upsampler = upsampler = Upsampler(platform, ClockSignal("sys"))
        #self.submodules.cdc = cdc = stream.ClockDomainCrossing([("data", port.data_width)], cd_from="sys", cd_to=clock_domain.name, depth=64)
        self.submodules.cdc = cdc = stream.ClockDomainCrossing([("data", 16)], cd_from="sys", cd_to=clock_domain.name, depth=64)
        #self.submodules.rrcosfilter = rrcosfilter = RRcosFilter(platform, clock_domain.clk)
        #self.submodules.rrcosfilter = rrcosfilter = RRcosFilter(platform, ClockSignal("sys")) # hacked clk
        #self.submodules.dac = dac = AlexandersDAC(platform, clock_domain, clock_domain2)
        #self.submodules.bbfilter = bbfilter = BbFilter(platform, ClockSignal("sys"))
        #self.submodules.bbfilter = bbfilter = BbFilter(platform, ClockSignal("dac"))
        self.submodules.bbfilter = bbfilter =  ClockDomainsRenamer("dac")(BbFilter(platform, clock_domain))
        self.submodules.dac = dac = NicksDAC(platform, clock_domain, clock_domain2)
        self.submodules.nullsink = nullsink = NullSink(platform)

        self.submodules.pipeline = Pipeline(
            dma,
            cdc,
            bbfilter,
            #upsampler,
            #cdc,
            #rrcosfilter,
            nullsink,
        )
        #self.comb += dma.source.connect(upsampler.sink)
        #self.comb += upsampler.source.connect(cdc.sink)
        #self.comb += cdc.source.connect(nullsink.sink)
        #self.comb += cdc.source.connect(rrcosfilter.sink)
        #self.comb += rrcosfilter.source.connect(dac.sink)

        #self.comb += self.dma.source.connect(self.cdc.sink)
        #self.comb += cdc.source.connect(rrcosfilter.sink)
        #self.comb += cdc.source.connect(upsampler.sink)
        #self.comb += rrcosfilter.source.connect(upsampler.sink)



        #self.comb += cdc.source.connect(dac.sink)
        #self.comb += upsampler.source.connect(dac.sink)

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


        if own_dma == 1:
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
