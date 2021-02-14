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

print("NICK DEBUG")
time.sleep(2)
#The control word is latched
#on the rising edge of CW. CW is independent of the DAC
#clock. The DAC clock can always remain running, when
#the control word is written to the DAC.
wb.write(wb.regs.dac_cw.addr, 1)
wb.write(wb.regs.dac_cw.addr, 0)

#writhing the same data to both registers to be "safe"
wb.write(wb.regs.dac_data_a_storage.addr, 0x120)
wb.write(wb.regs.dac_data_b_storage.addr, 0x120)

#latch in the settings.
wb.write(wb.regs.dac_cw.addr, 1)


#now we should be able to stream to the DAC

for i in range(1, 42):
    print(f"Writing dac_data_a_storage {i}")
    wb.write(wb.regs.dac_data_a_storage.addr, i)
    time.sleep(0.1)


wb.close()
