import argparse
import json
import sys
import time
from enum import Enum
from subprocess import run
from time import perf_counter

import pylink
from intelhex import IntelHex
from pylink.registers import SelectRegisterBits

# Access point ID for the CTRL-AP
CTRLAP_ID = 4
# Address of the DP SELECT register
DP_REG_SELECT = 0x8

# Offset of the BOOTMODE.OPCODE field
BOOTMODE_OPCODE_POS = 1
# BOOTMODE.OPCODE for DEBUGWAIT
OPCODE_DEBUGWAIT = 0x2


class Address(int, Enum):
    """Enum for addresses used in the CTRLAP"""

    RESET = 0x0
    READY = 0x4
    BOOTMODE = 0x20
    BOOTSTATUS = 0x38

    @property
    def bank(self) -> int:
        return reg_bank_get(self)

    @property
    def index(self) -> int:
        return reg_index_get(self)


def reg_bank_get(address: int) -> int:
    return (address & 0xF0) >> 4


def reg_index_get(address: int) -> int:
    return (address & 0x0F) // 4


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--dev-id", default="")
    parser.add_argument(
        "--firmware",
        type=argparse.FileType("r", encoding="utf-8"),
        required=True,
    )
    parser.add_argument("--debugwait-reset", action="store_true", default=False)
    args = parser.parse_args()

    jlink = pylink.JLink()

    dev_id = args.dev_id
    connected_devices = [str(info.SerialNumber) for info in jlink.connected_emulators()]
    if len(connected_devices) == 0:
        sys.exit("No J-Link devices are connected")
    elif len(connected_devices) == 1 and not dev_id:
        dev_id = connected_devices[0]
    elif dev_id not in connected_devices:
        sys.exit(
            f"Unknown J-Link serial number. Choose from: {', '.join(connected_devices)}"
        )

    jlink.open(dev_id)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.set_speed(speed=1000)
    jlink.exec_command(f"CORESIGHT_SetIndexAHBAPToUse = {0}")
    jlink.exec_command("EnableLowPowerHandlingMode ")
    jlink.coresight_configure()

    print("Waiting for READY = 0")

    # Wait for READY to indicate CTRL-AP readiness
    ready = 1
    t_start = perf_counter()
    while True:
        jlink.coresight_configure()
        jlink.coresight_write(
            reg=reg_index_get(DP_REG_SELECT),
            data=int.from_bytes(
                SelectRegisterBits(APSEL=CTRLAP_ID, APBANKSEL=Address.READY.bank),
                "little",
            ),
            ap=False,
        )
        ready = jlink.coresight_read(reg=Address.READY.index, ap=True)
        if ready == 0 or (perf_counter() - t_start) >= 1.000:
            break

    if ready != 0:
        sys.exit("Timed out waiting for READY == 0")

    if args.debugwait_reset:
        print(f"Setting BOOTMODE.OPCODE = {OPCODE_DEBUGWAIT} (DEBUGWAIT)")

        jlink.coresight_write(
            reg=reg_index_get(DP_REG_SELECT),
            data=int.from_bytes(
                SelectRegisterBits(APSEL=CTRLAP_ID, APBANKSEL=Address.BOOTMODE.bank),
                "little",
            ),
            ap=False,
        )
        jlink.coresight_write(
            reg=Address.BOOTMODE.index,
            data=(OPCODE_DEBUGWAIT << BOOTMODE_OPCODE_POS),
            ap=True,
        )

        print("Setting RESET = 1 (trigger CTRL-AP reset)")

        jlink.coresight_write(
            reg=reg_index_get(DP_REG_SELECT),
            data=int.from_bytes(
                SelectRegisterBits(APSEL=CTRLAP_ID, APBANKSEL=Address.RESET.bank), "little"
            ),
            ap=False,
        )
        jlink.coresight_write(reg=Address.RESET.index, data=1, ap=True)

    print("Waiting for BOOTSTATUS != 0")

    bootstatus = 0
    t_start = perf_counter()
    while True:
        jlink.coresight_configure()
        jlink.coresight_write(
            reg=reg_index_get(DP_REG_SELECT),
            data=int.from_bytes(
                SelectRegisterBits(APSEL=CTRLAP_ID, APBANKSEL=Address.BOOTSTATUS.bank),
                "little",
            ),
            ap=False,
        )
        bootstatus = jlink.coresight_read(reg=Address.BOOTSTATUS.index, ap=True)
        if bootstatus != 0 or (perf_counter() - t_start) >= 1.000:
            break

    if bootstatus == 0:
        sys.exit("Timed out waiting for BOOTSTATUS != 0")

    print(f"BOOTSTATUS = 0x{bootstatus:09_x}")

    print("Connecting to CPU")
    jlink.connect(chip_name="Cortex-M33")

    print("Halting CPU")
    jlink.halt()

    chunk_size = 8192

    ih = IntelHex(args.firmware)
    for start_addr, end_addr in ih.segments():
        segment_data = [ih[addr] for addr in range(start_addr, end_addr)]
        for offset in range(0, len(segment_data), chunk_size):
            chunk_data = segment_data[offset : offset + chunk_size]
            chunk_addr = start_addr + offset
            print(
                f"Write chunk [0x{chunk_addr:09_x}, 0x{chunk_addr + len(chunk_data):09_x})"
            )
            jlink.memory_write8(chunk_addr, chunk_data)
            chunk_data_readback = jlink.memory_read8(chunk_addr, len(chunk_data))
            if chunk_data != chunk_data_readback:
                sys.exit(
                    f"Readback failed for chunk [0x{chunk_addr:09_x}, 0x{chunk_addr + len(chunk_data):09_x})"
                )

    # Assume that vector table is at the first address
    vector_table_addr = ih.minaddr()
    stack_pointer, program_counter = jlink.memory_read32(vector_table_addr, 2)

    print(f"Set PC = 0x{program_counter:09_x}")

    # Set target to Debug Core Register Data
    jlink.coresight_write(1, 0xE000EDF8, ap=True)
    # Set PC
    jlink.coresight_write(3, program_counter, ap=True)
    # Set target to Debug Core Reg Sel
    jlink.coresight_write(1, 0xE000EDF4, ap=True)
    # Write to select the PC
    jlink.coresight_write(3, 0x1000F, ap=True)
    # Set target to Debug Halt Control
    jlink.coresight_write(1, 0xE000EDF0, ap=True)
    # Dummy reads
    jlink.coresight_read(3, ap=True)
    jlink.coresight_read(3, ap=True)

    print(f"Set SP = 0x{stack_pointer:09_x}")

    # Set target to Debug Core Register Data
    jlink.coresight_write(1, 0xE000EDF8, ap=True)
    # SP = stack_pointer
    jlink.coresight_write(3, stack_pointer, ap=True)
    # Set target to Debug Core Reg Sel
    jlink.coresight_write(1, 0xE000EDF4, ap=True)
    # Write to select SP
    jlink.coresight_write(3, 0x1000D, ap=True)
    # Set target to Debug Halt Control
    jlink.coresight_write(1, 0xE000EDF0, ap=True)
    # Dummy reads
    jlink.coresight_read(3, ap=True)
    jlink.coresight_read(3, ap=True)

    print(f"Set VTOR = 0x{vector_table_addr:09_x}")

    # VTOR = vector_table_addr
    jlink.memory_write32(0xE000ED08, [vector_table_addr])

    print("Check if CPUCONF.CPUWAIT != 0")

    cpuwait = jlink.memory_read32(0x5201150C, 1)[0]
    if cpuwait:
        print("Set CPUCONF.CPUWAIT = 0")

        # CPUCONF.CPUWAIT = 0
        jlink.memory_write32(0x5201150C, [0])
    else:
        print("CPUCONF.CPUWAIT was already 0")

    print("Starting CPU")
    jlink.restart()

if __name__ == "__main__":
    main()
