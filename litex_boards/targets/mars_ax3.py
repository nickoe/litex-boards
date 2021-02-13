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

from litex_boards.platforms import mars_ax3
from litex.build.xilinx.vivado import vivado_build_args, vivado_build_argdict
from litex.build.io import DifferentialOutput

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.spi_flash import SpiFlash

from litex.soc.interconnect.csr import *


from litedram.modules import NT5CC128M16
from litedram.phy import s7ddrphy

#from liteeth.phy.mii import LiteEthPHYMII

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys       = ClockDomain()
        self.clock_domains.cd_sys4x     = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_idelay    = ClockDomain()
        self.clock_domains.cd_dac       = ClockDomain()
        self.clock_domains.cd_dac_180   = ClockDomain()
        #self.clock_domains.cd_eth       = ClockDomain()

        # # #
        ## TODO recheck these, changed from 100 MHz to 50MHz  arty to marx ax3
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset") | self.rst)
        pll.register_clkin(platform.request("clk50"), 50e6)
        pll.create_clkout(self.cd_sys,       sys_clk_freq)
        pll.create_clkout(self.cd_sys4x,     4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay,    200e6)
        pll.create_clkout(self.cd_dac,       10e6)
        pll.create_clkout(self.cd_dac_180,   10e6, phase=180)

        #pll.create_clkout(self.cd_eth,       25e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)

        #self.comb += platform.request("eth_ref_clk").eq(self.cd_eth.clk)



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


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    mem_map = {**SoCCore.mem_map, **{"spiflash": 0x20000000}}
    def __init__(self, toolchain="vivado", spiflash="spiflash_1x", sys_clk_freq=int(100e6), with_ethernet=False, with_etherbone=False, eth_ip="192.168.1.50", ident_version=True, **kwargs):
        platform = mars_ax3.Platform(toolchain=toolchain)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq,
            ident          = "LiteX SoC on Mars AX3 (IQFPGA) AAUAST6",
            ident_version  = ident_version,
            **kwargs)



        sys_clk_freq = int(100e6)
        # SoCSDRAM ---------------------------------------------------------------------------------
        #SoCSDRAM.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs)

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
                l2_cache_reverse        = True
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
        
        #bios_size = 0x8000
        #self.flash_boot_address = self.mem_map["spiflash"]+platform.gateware_size+bios_size
        #define_flash_constants(self)

        # Mars AX3 specific stuff
        self.comb += platform.request("ddr3_vsel").eq(0)

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
        self.comb += dac_plat.clkx_test.eq(self.crg.cd_dac.clk)
        self.comb += dac_plat.clkx_test2.eq(self.crg.cd_dac_180.clk)



        self.add_jtagbone()


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on foo")
    parser.add_argument("--toolchain",        default="vivado",                 help="Toolchain use to build (default: vivado)")
    parser.add_argument("--build",            action="store_true",              help="Build bitstream")
    parser.add_argument("--load",             action="store_true",              help="Load bitstream")
    parser.add_argument("--sys-clk-freq",     default=100e6,                    help="System clock frequency (default: 50MHz)")
    #ethopts = parser.add_mutually_exclusive_group()
    #ethopts.add_argument("--with-ethernet",   action="store_true",              help="Enable Ethernet support")
    #ethopts.add_argument("--with-etherbone",  action="store_true",              help="Enable Etherbone support")
    #parser.add_argument("--eth-ip",           default="192.168.1.50", type=str, help="Ethernet/Etherbone IP address")
    sdopts = parser.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",  action="store_true",              help="Enable SPI-mode SDCard support")
    sdopts.add_argument("--with-sdcard",      action="store_true",              help="Enable SDCard support")
    parser.add_argument("--no-ident-version", action="store_false",             help="Disable build time output")
    builder_args(parser)
    soc_sdram_args(parser)
    vivado_build_args(parser)
    args = parser.parse_args()

    soc = BaseSoC(
        toolchain      = args.toolchain,
        sys_clk_freq   = int(float(args.sys_clk_freq)),
        #with_ethernet  = args.with_ethernet,
        #with_etherbone = args.with_etherbone,
        #eth_ip         = args.eth_ip,
        ident_version  = args.no_ident_version,
        **soc_sdram_argdict(args)
    )
    #soc.platform.add_extension(arty._sdcard_pmod_io)
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()
    builder = Builder(soc, **builder_argdict(args))
    builder_kwargs = vivado_build_argdict(args) if args.toolchain == "vivado" else {}
    builder.build(**builder_kwargs, run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
