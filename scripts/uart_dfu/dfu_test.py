import subprocess
import west.app.main
import os
import os.path
import shutil
from uart_dfu import prepare

BUCKET_ADDRESS = "s3://dfubucket/"
NCS_BASE = os.path.realpath(os.path.join(os.getenv("ZEPHYR_BASE"), ".."))
NRF91_SAMPLE_PATH = "nrf/samples/nrf9160/uart_application_update"
NRF91_BIN_BASENAME = "app_update_nrf9160"
NRF52_SAMPLE_PATH = "nrf/samples/bluetooth/peripheral_uart_dfu"
NRF52_BIN_BASENAME = "app_update_nrf52840"
LOADER_SCRIPT_PATH = os.path.join(NRF52_SAMPLE_PATH, "scripts")
LOADER_HEX_PATH = "sfts_loader_nrf52840dk_buttonless.hex"
BIN_PATH = "build/zephyr/app_update.bin"
TMP_DIR = os.path.join(os.path.realpath(os.path.dirname(__file__)), "bin")

def west_build(sample_path, board):
        print(f"Building {sample_path}.")
        p = subprocess.run(["west", "build", "-b", board],
                           cwd=sample_path,
                           capture_output=True)
        p.check_returncode()
        return os.path.join(sample_path, BIN_PATH)


def aws_upload(path_in, path_out=None):
        print(f"Uploading {path_in}.")
        if path_out is not None:
                out = BUCKET_ADDRESS + path_out
        else:
                out = BUCKET_ADDRESS
        args = ["aws", "s3", "cp", path_in, out]
        p = subprocess.run(args, capture_output=True)
        p.check_returncode()


def loader_flash(bin_path):
        args = ["python", "sfts_prep.py", LOADER_HEX_PATH, bin_path]
        script_path = os.path.join(NCS_BASE, LOADER_SCRIPT_PATH)
        p = subprocess.run(args, cwd=script_path, capture_output=True)
        p.check_returncode()

# def image_create(sample_path, board, )

def images_create():
        if not os.path.exists(TMP_DIR):
                os.mkdir(TMP_DIR)
        nrf91_path = os.path.join(NCS_BASE, NRF91_SAMPLE_PATH)
        nrf52_path = os.path.join(NCS_BASE, NRF52_SAMPLE_PATH)
        nrf91_zephyr_bin = west_build(nrf91_path, "nrf9160dk_nrf9160ns")
        nrf52_zephyr_bin = west_build(nrf52_path, "nrf9160dk_nrf52840")
        nrf91_bin = os.path.join(TMP_DIR, NRF91_BIN_BASENAME + ".bin")
        nrf91_uart_bin = os.path.join(TMP_DIR, NRF91_BIN_BASENAME + "_uart.bin")
        nrf52_bin = os.path.join(TMP_DIR, NRF52_BIN_BASENAME + ".bin")
        nrf52_uart_bin = os.path.join(TMP_DIR, NRF52_BIN_BASENAME + "_uart.bin")
        shutil.copy(nrf91_zephyr_bin, nrf91_bin)
        print(f"Wrote {nrf91_bin}")
        shutil.copy(nrf52_zephyr_bin, nrf52_bin)
        print(f"Wrote {nrf52_bin}")
        prepare(nrf91_bin, nrf91_uart_bin)
        prepare(nrf52_bin, nrf52_uart_bin)
        return (nrf91_bin, nrf91_uart_bin), (nrf52_bin, nrf52_uart_bin)


def lte_dfu_test_run():
        (nrf91_bin, _), (_, nrf52_uart_bin) = images_create()
        aws_upload(nrf91_bin)
        aws_upload(nrf52_uart_bin)


def ble_dfu_test_run():
        (_, nrf91_uart_bin), (nrf52_bin, _) = images_create()
        loader_flash(nrf52_bin)


if __name__ == "__main__":
        ble_dfu_test_run()
        # lte_dfu_test_run()