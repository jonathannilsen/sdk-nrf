import inspect
import argparse
import copy
import os.path
import sys

from uart_dfu_protocol import UartDfuClient, UartDfuProtocolError


UART_HEADER_MAGIC = bytearray((0x85, 0xf3, 0xd8, 0x3a))
UART_DEFAULT_BAUDRATE = 115200


def error_exit(msg):
    print(msg, file=sys.stderr)
    sys.exit(1)


def prepare(path_in : str, path_out : str):
    if not os.path.exists(path_in):
        error_exit(f"File {path_in} does not exist.")
    if not os.path.exists(os.path.dirname(path_out)):
        error_exit(f"Directory {os.path.dirname(path_out)} does not exist.")
    with open(path_in, "rb") as f_in, open(path_out, "wb") as f_out:
        f_out.write(UART_HEADER_MAGIC)
        f_out.write(f_in.read())
    print(f"Wrote file {path_out}.")


def upload(path_in : str, port : str, baudrate : int=UART_DEFAULT_BAUDRATE):
    if not os.path.exists(path_in):
        error_exit(f"File {path_in} does not exist.")    
    try:
        with open(path_in, "rb") as f:
            image_data = f.read()
        print(f"Connecting to {port}@{baudrate}")
        client = UartDfuClient(port, baudrate)
        client.update(image_data)
        print(f"Successfully uploaded {path_in} to {port}@{baudrate}.")
    except UartDfuProtocolError as e:
        print(f"Protocol error: {e}")
    except Exception as e:
        print(f"Unhandled exception: {e}")


def download(path_out : str, port : str, baudrate : int=UART_DEFAULT_BAUDRATE):
    error_exit("Download is not yet implemented")


def subcommand_parser_add(subparsers, func):
    sig = inspect.signature(func)
    parser = subparsers.add_parser(func.__name__)
    empty_to_none = lambda p: p if p != inspect.Parameter.empty else None
    for param in sig.parameters.values():
        if param.default != inspect.Parameter.empty:
            parser.add_argument(f"--{param.name}",
                                default=param.default,
                                type=empty_to_none(param.annotation))
        else:
            parser.add_argument(param.name,
                                type=empty_to_none(param.annotation))
    parser.set_defaults(func=func)


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers() 
    for subcommand in [prepare, upload, download]:
        subcommand_parser_add(subparsers, subcommand)
    args = parser.parse_args()
    func_args = copy.copy(vars(args))
    del func_args["func"]
    args.func(**func_args)

if __name__ == "__main__":
    main()