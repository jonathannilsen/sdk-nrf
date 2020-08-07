#!/usr/bin/env python3
import pylink # pip install pylink-square
from sys import argv, exit

ADDR_FIRMWARE       = 0x00000
ADDR_SFTS_SIZE      = 0x50000
ADDR_SFTS_DATA      = 0x50008
JLINK_TARGET_DEVICE = 'nRF52840_xxAA'
USAGE_OPTION_VERIFY = '--verify'
USAGE_MESSAGE       = '''
This script will prepare an SFTS client board to transmit a file over BLE.
Usage:
  python {0} <firmware.hex> <data.bin>
  python {0} {1}
'''[1:-1]

def connect():
    print('Connecting to JLink...')
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(JLINK_TARGET_DEVICE)

    return jlink

if __name__ == '__main__':
    if len(argv) == 2 and argv[1] == USAGE_OPTION_VERIFY:
        jlink = connect()
        bytes = jlink.memory_read8(ADDR_SFTS_SIZE, 16)

        print(f'0x{ADDR_SFTS_SIZE:08x}:', *(f'{b:02x}' for b in bytes), '...')

    elif len(argv) == 3:
        jlink = connect()
        firmware_hex, data_bin = argv[1:]

        print(f'Flashing {firmware_hex:}...')
        jlink.flash_file(firmware_hex, ADDR_FIRMWARE);

        print(f'Flashing {data_bin:}...')
        jlink.flash_file(data_bin, ADDR_SFTS_DATA)

        size, crc = 0, 0xFFFFFFFF
        with open(data_bin, 'rb') as file:
            while True:
                try:
                    byte = file.read(1)[0]
                    size += 1
                    crc ^= byte
                    for _ in range(8):
                        crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
                except IndexError:
                    break

        print('Writing length and CRC...')
        jlink.flash_write32(ADDR_SFTS_SIZE, [size, ~crc])

        print('Resetting...')
        jlink.reset(halt=False)

        print('Done')

    else:
        print(USAGE_MESSAGE.format(argv[0], USAGE_OPTION_VERIFY))
