#!/usr/bin/env python3
import time
from litex import RemoteClient

wb = RemoteClient()
wb.open()

# # #

# get identifier
fpga_id = ""
for i in range(256):
    c = chr(wb.read(wb.bases.identifier_mem + 4*i) & 0xff)
    fpga_id += c
    if c == "\0":
        break
print("fpga_id: " + fpga_id)

# # #

# Trigger a reset of the SoC
wb.regs.ctrl_reset.write(1)

# Dump all CSR registers of the SoC
for name, reg in wb.regs.__dict__.items():
    print("0x{:08x} : 0x{:08x} {}".format(reg.addr, reg.read(), name))

time.sleep(1)
print("NICK DEBUG")
for i in range(1, 42):
    print(f"Writing dac_data_a_storage {i}")
    wb.write(wb.regs.dac_data_a_storage.addr, i)

    time.sleep(1)


wb.close()
