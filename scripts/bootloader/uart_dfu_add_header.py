#!/usr/bin/env python3
#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic

import argparse
import os.path
import sys

""" Magic value to place at the start of the UART DFU binary. """
UART_HEADER_MAGIC = bytearray((0x85, 0xf3, 0xd8, 0x3a))

def prepare(path_in : str, path_out : str):
    """ Prepend UART DFU magic number to the given file """
    with open(path_in, "rb") as f_in, open(path_out, "wb") as f_out:
        f_out.write(UART_HEADER_MAGIC)
        f_out.write(f_in.read())

if __name__ == "__main__":
    try:
        p = argparse.ArgumentParser(description="Prepend UART DFU magic number to a BIN file.")
        p.add_argument("path_in", type=str, help="Input .bin file")
        p.add_argument("path_out", type=str, help="Output .bin file")
        a = p.parse_args()
        prepare(a.path_in, a.path_out)
    except Exception as e:
        print(e, file=sys.stderr)
        sys.exit(1)