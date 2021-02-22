# This file is Copyright (c) 2021 Nick Østergaard
# License: MIT

from litex.build.generic_platform import *
from litex.build.openocd import OpenOCD
from litex.build.xilinx import XilinxPlatform, XC3SProg, VivadoProgrammer

_io = [
    ("user_led", 0, Pins("M16"), IOStandard("LVCMOS18")),
    ("user_led", 1, Pins("M17"), IOStandard("LVCMOS18")),
    ("user_led", 2, Pins("L18"), IOStandard("LVCMOS18")),
    ("user_led", 3, Pins("M18"), IOStandard("LVCMOS18")),

    ("clk50", 0, Pins("P17"), IOStandard("LVCMOS18")),

    # not really connected, but arty soc example uses this -- I think
    ("cpu_reset", 0, Pins("C2"), IOStandard("LVCMOS33")),



    # TODO pmod, serial, flash, sdcard, dac?

    ("serial", 0,
        Subsignal("tx", Pins("U13")),
        Subsignal("rx", Pins("U11")),
        IOStandard("LVCMOS18")),
    #The FLASH_CLK_FPGA_CCLK isconnected to one pin on the header, and two pads on the artix7...??
    #FLASH_CLK_FPGA_CCLK	E9	CCLK_0	 182	FPGA config clock
    #FLASH_CLK_FPGA_CCLK	R10	IO_25_14 182	connected to a user IO for flash access after configuration
    ("spiflash_4x", 0,  # clock needs to be accessed through STARTUPE2 (nick has no idea what this means)
        Subsignal("cs_n", Pins("L13")),
        Subsignal("clk", Pins("R10")),
        Subsignal("dq", Pins("K17", "K18", "L14", "M14")),
        IOStandard("LVCMOS18")
    ),
    ("spiflash_1x", 0,  # clock needs to be accessed through STARTUPE2 (nick has no idea what this means)
        Subsignal("cs_n", Pins("L13")),
        Subsignal("clk", Pins("R10")),
        Subsignal("mosi", Pins("K17")),
        Subsignal("miso", Pins("K18")),
        Subsignal("wp", Pins("L14")),
        Subsignal("hold", Pins("M14")),
        IOStandard("LVCMOS18")
    ),
    # DDR3 SDRAM
    # TODO remember to add "on die termination", see 2.14.3 Termination
    ("ddram", 0,
     Subsignal("a", Pins(
         "J17 J14 J18 D18 J13 E17 K13 E18",
         "H17 F18 G16 G18 H16 G17 H15"),
               IOStandard("SSTL15")),
     Subsignal("ba", Pins("D17 H14 K15"), IOStandard("SSTL15")),
     Subsignal("ras_n", Pins("F15"), IOStandard("SSTL15")),
     Subsignal("cas_n", Pins("F16"), IOStandard("SSTL15")),
     Subsignal("we_n", Pins("J15"), IOStandard("SSTL15")),
     #Subsignal("cs_n", Pins(""), IOStandard("SSTL15")), # 100R pulldown on board
     Subsignal("dm", Pins("D15 D12"), IOStandard("SSTL15")),
     Subsignal("dq", Pins(
         "A18 E16 A15 E15 B18 B17 A16 B16",
         "B14 C14 B13 D14 F13 A11 F14 B11"),
               IOStandard("SSTL15"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("dqs_p", Pins("A13 C12"),
               IOStandard("DIFF_SSTL15"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("dqs_n", Pins("A14 B12"),
               IOStandard("DIFF_SSTL15"),
               Misc("IN_TERM=UNTUNED_SPLIT_40")),
     Subsignal("clk_p", Pins("C16"), IOStandard("DIFF_SSTL15")),
     Subsignal("clk_n", Pins("C17"), IOStandard("DIFF_SSTL15")),
     Subsignal("cke", Pins("G14"), IOStandard("SSTL15")),
     Subsignal("odt", Pins("K16"), IOStandard("SSTL15")),
     Subsignal("reset_n", Pins("G13"), IOStandard("LVCMOS15")),
     Misc("SLEW=FAST"),
     ),
    # 2.14.5
    # DDR3 Low Voltage Operation
    # Low voltage operation for the DDR3 SDRAM is available only for modules revision 2 and newer.
    # The default voltage of the DDR3 is 1.5 V. In order to enable low voltage mode (1.35 V), DDR3_VSEL (pin D9)
    # must be driven logic 0 by the FPGA logic, and a memory voltage of 1.35 V must be selected in the Memory
    # Interface Generator (MIG) parameters in Vivado.
    # For 1.5 V operation, DDR3_VSEL must be set to high impedance (not driven logic 1).
    #("ddr3_vsel", 0, Pins("D9"), IOStandard("LVCMOS33")),
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
        Subsignal("clkx_p", Pins("K5"), IOStandard("LVCMOS33")),
        Subsignal("clkx_n", Pins("L4"), IOStandard("LVCMOS33")),
        Subsignal("clkx_test", Pins("F6"), IOStandard("LVCMOS33")),
        Subsignal("clkx_test2", Pins("G2"), IOStandard("LVCMOS33")),
    ),
    # RGMII Ethernet
    ("eth_clocks", 0,
     Subsignal("tx", Pins("N16")),
     Subsignal("rx", Pins("T14")),
     IOStandard("LVCMOS18")
     ),
    ("eth", 0,
     Subsignal("rst_n", Pins("M13"), IOStandard("LVCMOS18")),
     Subsignal("int_n", Pins("T15")),
     Subsignal("mdio", Pins("N14")),
     Subsignal("mdc", Pins("P14")),
     Subsignal("rx_ctl", Pins("R16")),
     Subsignal("rx_data", Pins("U16 V17 V15 V16")),
     Subsignal("tx_ctl", Pins("T16")),
     Subsignal("tx_data", Pins("R18 T18 U17 U18")),
     IOStandard("LVCMOS18")
     ),

    '''
    # PL_Gigabit_Ethernet
    set_property SLEW FAST [get_ports ETH_TXC]
    set_property SLEW FAST [get_ports ETH_TX_CTL]
    set_property SLEW FAST [get_ports {ETH_TXD[0]}]
    set_property SLEW FAST [get_ports {ETH_TXD[1]}]
    set_property SLEW FAST [get_ports {ETH_TXD[2]}]
    set_property SLEW FAST [get_ports {ETH_TXD[3]}]
    set_property -dict {PACKAGE_PIN P14   IOSTANDARD LVCMOS18  } [get_ports {ETH_MDC}]
    set_property -dict {PACKAGE_PIN U16   IOSTANDARD LVCMOS18  } [get_ports {ETH_RXD[0]}]
    set_property -dict {PACKAGE_PIN V17   IOSTANDARD LVCMOS18  } [get_ports {ETH_RXD[1]}]
    set_property -dict {PACKAGE_PIN V15   IOSTANDARD LVCMOS18  } [get_ports {ETH_RXD[2]}]
    set_property -dict {PACKAGE_PIN V16   IOSTANDARD LVCMOS18  } [get_ports {ETH_RXD[3]}]
    set_property -dict {PACKAGE_PIN T14   IOSTANDARD LVCMOS18  } [get_ports {ETH_RXC}]
    set_property -dict {PACKAGE_PIN R18   IOSTANDARD LVCMOS18  } [get_ports {ETH_TXD[0]}]
    set_property -dict {PACKAGE_PIN T18   IOSTANDARD LVCMOS18  } [get_ports {ETH_TXD[1]}]
    set_property -dict {PACKAGE_PIN U17   IOSTANDARD LVCMOS18  } [get_ports {ETH_TXD[2]}]
    set_property -dict {PACKAGE_PIN U18   IOSTANDARD LVCMOS18  } [get_ports {ETH_TXD[3]}]
    set_property -dict {PACKAGE_PIN N16   IOSTANDARD LVCMOS18  } [get_ports {ETH_TXC}]
    set_property -dict {PACKAGE_PIN N14   IOSTANDARD LVCMOS18  } [get_ports {ETH_MDIO}]
    set_property -dict {PACKAGE_PIN T15   IOSTANDARD LVCMOS18  } [get_ports {ETH_INT_N}]
    set_property -dict {PACKAGE_PIN M13   IOSTANDARD LVCMOS18  } [get_ports {ETH_RST_N}]
    set_property -dict {PACKAGE_PIN R16   IOSTANDARD LVCMOS18  } [get_ports {ETH_RX_CTL}]
    set_property -dict {PACKAGE_PIN T16   IOSTANDARD LVCMOS18  } [get_ports {ETH_TX_CTL}]
    '''
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
        if toolchain == "vivado":
            XilinxPlatform.__init__(self, "xc7a35t-csg324-1", _io, _connectors, toolchain=toolchain)
            self.toolchain.bitstream_commands = \
                ["set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]"]
            self.toolchain.additional_commands = \
                ["write_cfgmem -force -format bin -interface spix4 -size 16 "
                 "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"]
            #self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 34]")
            #self.add_platform_command("set_property INTERNAL_VREF 0.675 [get_iobanks 15]") # LV
            self.add_platform_command("set_property INTERNAL_VREF 0.750 [get_iobanks 15]")  # 1.5V
            self.programmer = programmer

            # hack to make it rout the the clock, bad chice from enclustra, or did I d something wrong?
            #Phase 1.2 IO Placement/ Clock Placement/ Build Placer Device
            #WARNING: [Place 30-574] Poor placement for routing between an IO pin and BUFG. This is normally an ERROR but the CLOCK_DEDICATED_ROUTE constraint is set to FALSE allowing your design to continue. The use of this override is highly discouraged as it may lead to very poor timing results. It is recommended that this error condition be corrected in the design.
            #self.add_platform_command("set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets eth_clocks_tx_IBUF]")


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



