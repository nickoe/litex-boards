# This file is Copyright (c) 2021 Nick Østergaard
# License: MIT

from litex.build.generic_platform import *
from litex.build.openocd import OpenOCD
from litex.build.xilinx import XilinxPlatform, XC3SProg, VivadoProgrammer

_io = [
    ("user_led", 0, Pins("M16"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("M17"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("L18"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("M18"), IOStandard("LVCMOS33")),

    ("clk50", 0, Pins("P17"), IOStandard("LVCMOS33")),

    # not really connected, but arty soc example uses this -- I think
    ("cpu_reset", 0, Pins("C2"), IOStandard("LVCMOS33")),



    # TODO pmod, serial, flash, sdcard, dac?

    ("serial", 0,
        Subsignal("tx", Pins("U13")),
        Subsignal("rx", Pins("U11")),
        IOStandard("LVCMOS33")),

    ("spiflash_4x", 0,  # clock needs to be accessed through STARTUPE2 (nick has no idea what this means)
        Subsignal("cs_n", Pins("L13")),
        Subsignal("dq", Pins("K17", "K18", "L14", "M14")),
        IOStandard("LVCMOS33")
    ),
    ("spiflash_1x", 0,  # clock needs to be accessed through STARTUPE2 (nick has no idea what this means)
        Subsignal("cs_n", Pins("L13")),
        Subsignal("mosi", Pins("K17")),
        Subsignal("miso", Pins("K18")),
        Subsignal("wp", Pins("L14")),
        Subsignal("hold", Pins("M14")),
        IOStandard("LVCMOS33")
    ),
    # DDR3 SDRAM
    # TODO remember to add "on die termination", see 2.14.3 Termination
    ("ddram", 0,
     Subsignal("a", Pins(
         "J17 J14 J18 D18 J13 E17 K13 E18",
         "H17 F18 G16 G18 H16 G17 H15"),
               IOStandard("SSTL135")),
     Subsignal("ba", Pins("D17 H14 K15"), IOStandard("SSTL135")),
     Subsignal("ras_n", Pins("F15"), IOStandard("SSTL135")),
     Subsignal("cas_n", Pins("F16"), IOStandard("SSTL135")),
     Subsignal("we_n", Pins("J15"), IOStandard("SSTL135")),
     #Subsignal("cs_n", Pins(""), IOStandard("SSTL135")), # 100R pulldown on board
     Subsignal("dm", Pins("D15 D12"), IOStandard("SSTL135")),
     Subsignal("dq", Pins(
         "A18 E16 A15 E15 B18 B17 A16 B16",
         "B14 C14 B13 D14 F13 A11 F14 B11"),
               IOStandard("SSTL135"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("dqs_p", Pins("A13 C12"),
               IOStandard("DIFF_SSTL135"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("dqs_n", Pins("A14 B12"),
               IOStandard("DIFF_SSTL135"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("clk_p", Pins("C16"), IOStandard("DIFF_SSTL135")),
     Subsignal("clk_n", Pins("C17"), IOStandard("DIFF_SSTL135")),
     Subsignal("cke", Pins("G14"), IOStandard("SSTL135")),
     Subsignal("odt", Pins("K16"), IOStandard("SSTL135")),
     Subsignal("reset_n", Pins("G13"), IOStandard("SSTL135")),
     Misc("SLEW=FAST"),
     ),
    # 2.14.5
    # DDR3 Low Voltage Operation
    # Low voltage operation for the DDR3 SDRAM is available only for modules revision 2 and newer.
    # The default voltage of the DDR3 is 1.5 V. In order to enable low voltage mode (1.35 V), DDR3_VSEL (pin D9)
    # must be driven logic 0 by the FPGA logic, and a memory voltage of 1.35 V must be selected in the Memory
    # Interface Generator (MIG) parameters in Vivado.
    # For 1.5 V operation, DDR3_VSEL must be set to high impedance (not driven logic 1).
    ("ddr3_vsel", 0, Pins("D9"), IOStandard("LVCMOS33")),
    # SD carg via SPI connection
    ("spisdcard", 0,
     Subsignal("clk", Pins("B4")),
     Subsignal("mosi", Pins("B1"), Misc("PULLUP True")),
     Subsignal("cs_n", Pins("A1"), Misc("PULLUP True")),
     Subsignal("miso", Pins("C4"), Misc("PULLUP True")),
     Misc("SLEW=FAST"),
     IOStandard("LVCMOS33"),
     ),
    # MAX5854 Dual, 10-Bit, 165Msps, Current-Output DAC
    # https://www.maximintegrated.com/en/products/analog/data-converters/digital-to-analog-converters/MAX5854.html
    ("dac", 0,
        Subsignal("data_a", Pins("R1 T1 U1 V1 U4 U3 U2 V2 V7 V6"), IOStandard("LVCMOS33")),
        Subsignal("data_b", Pins("N5 P5 L1 M1 N2 N1 P4 P3 R6 R5"), IOStandard("LVCMOS33")),
        Subsignal("cw", Pins("L3"), IOStandard("LVCMOS33")),
    )
]

_connectors = []


# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    default_clk_name   = "clk50"
    default_clk_period = 1e9/50e6

    # From https://www.xilinx.com/support/documentation/user_guides/ug470_7Series_Config.pdf
    # 17536096 bits == 2192012 == 0x21728c -- Therefore 0x220000
    gateware_size = 0x220000

    # The bigger flash device introduced starting with revision 3 has bigger erase sectors (256 kB instead of 4 kB)
    # and the 4 kB/32 kB/64 kB erase commands are not supported anymore. Further, the programming buffer
    # is 512 bytes instead of 256 bytes. This may require adjustments of the programming algorithm.
    # We have Revision 3, so that is: Cypress (Spansion) S25FL512S 512 Mbit
    # https://www.cypress.com/file/177971/download
    # TODO, flash not tested
    spiflash_model = "s25fl512s"
    spiflash_read_dummy_bits = 6
    spiflash_clock_div = 4
    spiflash_total_size = int((512/8)*1024*1024) # 512Mbit
    spiflash_page_size = 512
    spiflash_sector_size = 256*1024

    def __init__(self, toolchain="vivado", programmer="openocd"):
        XilinxPlatform.__init__(self, "xc7a35t-csg324-1", _io, _connectors, toolchain=toolchain)
        self.toolchain.bitstream_commands = \
            ["set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]"]
        self.toolchain.additional_commands = \
            ["write_cfgmem -force -format bin -interface spix4 -size 16 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]
        #self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 34]")
        self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 15]")
        self.programmer = programmer


    def create_programmer(self):
        if self.programmer == "openocd":
            proxy="bscan_spi_{}.bit".format(self.device.split('-')[0])
            return OpenOCD(config="openocd_nick.cfg", flash_proxy_basename=proxy)
        else:
            raise ValueError("{} programmer is not supported"
                             .format(self.programmer))

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)
        from litex.build.xilinx import symbiflow
        self.add_period_constraint(self.lookup_request("clk10", loose=True), 1e9/10e6)



