#!/usr/bin/env python3
# Copyright (c) 2025 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

from __future__ import annotations

import abc
import argparse
import base64
import json
import struct
import subprocess
import time
from ctypes import c_char_p
from enum import Enum
from pathlib import Path
from tempfile import TemporaryDirectory
from textwrap import indent
from typing import Any
from zipfile import ZipFile

import cbor2
import serial
from intelhex import IntelHex
from west.commands import WestCommand


class NcsIronSideSEUpdate(WestCommand):
    def __init__(self) -> None:
        super().__init__(
            name="ncs-ironside-se-update",
            help="NCS IronSide SE update",
            description="Update IronSide SE.",
        )

    def do_add_parser(self, parser_adder: Any) -> argparse.ArgumentParser:
        parser = parser_adder.add_parser(self.name, help=self.help, description=self.description)
        parser.add_argument(
            "--zip",
            required=True,
            help="Path to IronSide SE release ZIP",
            type=argparse.FileType(mode="r"),
        )
        parser.add_argument(
            "--serial",
            dest="serial_number",
            type=str,
            help="Serial number (nrfutil mode)",
        )
        parser.add_argument(
            "--firmware-slot",
            help="Only update the given firmware slot instead of updating both slots",
            choices=[m.value for m in FirmwareSlot.__members__.values()],
            type=FirmwareSlot,
        )
        parser.add_argument(
            "--allow-erase",
            action="store_true",
            help="Allow erasing the device (required for debugger-based update)",
        )
        parser.add_argument(
            "--wait-time",
            type=float,
            help="Timeout in seconds to wait for the device to boot (nrfutil mode)",
            default=2.0,
        )

        parser.add_argument(
            "--smp-port",
            type=str,
            help="Serial port for SMP update (e.g. /dev/ttyACM0). "
            "When set, update is performed over SMP instead of nrfutil.",
        )
        parser.add_argument(
            "--smp-baud",
            type=int,
            default=115200,
            help="Baud rate for SMP serial (default: 115200)",
        )
        parser.add_argument(
            "--smp-chunk-size",
            type=int,
            default=256,
            help="Bytes per SMP upload request (default: 256)",
        )

        return parser

    def do_run(self, args: argparse.Namespace, unknown: list[str]) -> None:
        backend = self._create_backend(args)

        try:
            with TemporaryDirectory() as tmpdir, ZipFile(args.zip.name, "r") as zip_ref:
                zip_ref.extractall(tmpdir)
                update_dir = Path(tmpdir, "update")
                update_hex_files = self._select_slots(args.firmware_slot, update_dir)

                target_versions = {}
                for slot, update_file in update_hex_files:
                    if not update_file.exists():
                        self.die(
                            f"Update firmware file for {slot.description} ({slot.value}) "
                            f"not found in ZIP: {update_file.relative_to(update_dir)}"
                        )
                    target_versions[slot] = _load_update_version(update_file)

                fw_version_before = backend.read_versions()
                target_version_diff = self._fmt_versions(
                    before=fw_version_before,
                    after=target_versions,
                )
                self.inf(f"Updating IronSide SE firmware:\n{indent(target_version_diff, '  ')}\n")

                backend.prepare(update_dir)

                update_failed = False
                last_update_status = UpdateStatus.NONE

                for slot, update_file in update_hex_files:
                    slot_target_version = target_versions[slot]
                    self.inf(f"Updating {slot.description} ({slot.name}) to {slot_target_version}")

                    backend.program_update(slot, update_file)
                    backend.reset_and_wait()

                    slot_version = backend.read_versions()[slot]
                    last_update_status = backend.read_update_status()

                    self.dbg(f"  {slot.description}: {slot_version}")
                    if last_update_status.is_error:
                        self.dbg(
                            f"  Update status: {last_update_status.description} "
                            f"({last_update_status.name})"
                        )

                    if slot_version != slot_target_version:
                        update_failed = True
                        self.err(
                            f"Failed to update {slot.description} ({slot.name}) "
                            f"to {slot_target_version}"
                        )
                        break

                backend.cleanup()

                fw_version_after = backend.read_versions()
                version_str = self._fmt_versions(before=None, after=fw_version_after)
                self.inf(f"\nFinal firmware versions:\n{indent(version_str, '  ')}")

                if update_failed:
                    if last_update_status != UpdateStatus.NONE:
                        fail_status_str = (
                            f"{last_update_status.name} - {last_update_status.description}"
                        )
                    else:
                        fail_status_str = (
                            "The update status did not indicate why the failure happened"
                        )
                    self.die(fail_status_str)
        finally:
            backend.close()

    def _create_backend(self, args: argparse.Namespace) -> UpdateBackend:
        if args.smp_port:
            return SmpBackend(self, args.smp_port, args.smp_baud, args.smp_chunk_size)

        if not args.allow_erase:
            self.die("Unable to perform update without erasing the device, set '--allow-erase'")
        return NrfutilBackend(self, args.serial_number, args.wait_time)

    def _select_slots(
        self, firmware_slot: FirmwareSlot | None, update_dir: Path
    ) -> list[tuple[FirmwareSlot, Path]]:
        if firmware_slot:
            return [(firmware_slot, update_dir / firmware_slot.update_hex_name)]
        return [
            (FirmwareSlot.RSLOT, update_dir / FirmwareSlot.RSLOT.update_hex_name),
            (FirmwareSlot.USLOT, update_dir / FirmwareSlot.USLOT.update_hex_name),
        ]

    def _fmt_versions(
        self, *, before: dict[FirmwareSlot, str] | None, after: dict[FirmwareSlot, str]
    ) -> str:
        lines = []
        for slot in after:
            if before is not None:
                slot_versions = f"{before[slot]} -> {after[slot]}"
            else:
                slot_versions = after[slot]
            slot_name = f"{slot.description} ({slot.name})"
            lines.append(f"{slot_versions} - {slot_name}")
        return "\n".join(lines)


class UpdateStatus(int, Enum):
    NONE = 0xFFFF_FFFF
    INVALID_MANIFEST = 0xF000_0002
    STALE_FW = 0xF000_0003
    VERIFY_FAILURE = 0xF000_0005
    UROT_UPDATE_DISABLED = 0xF000_0007
    UROT_ACTIVATED = 0xF000_0008
    RECOVERY_ACTIVATED = 0xF000_0009
    RECOVERY_UPDATE_DISABLED = 0xF000_000A

    @property
    def is_error(self) -> bool:
        return self not in (
            UpdateStatus.NONE,
            UpdateStatus.UROT_ACTIVATED,
            UpdateStatus.RECOVERY_ACTIVATED,
        )

    @property
    def description(self) -> str:
        match self:
            case UpdateStatus.NONE:
                return "no update status"
            case UpdateStatus.INVALID_MANIFEST:
                return "the manifest in the update blob contains invalid data"
            case UpdateStatus.STALE_FW:
                return (
                    "the firmware is older than the installed firmware, "
                    "and firmware downgrades are disabled on the device"
                )
            case UpdateStatus.VERIFY_FAILURE:
                return (
                    "failed to verify the signature of the firmware, possibly "
                    "due to it being signed using different keys than those installed in the device"
                )
            case UpdateStatus.UROT_UPDATE_DISABLED:
                return "updates of the IronSide SE firmware are disabled on the device"
            case UpdateStatus.UROT_ACTIVATED:
                return "the IronSide SE firmware was successfully updated"
            case UpdateStatus.RECOVERY_ACTIVATED:
                return "the IronSide SE Recovery firmware was successfully updated"
            case UpdateStatus.RECOVERY_UPDATE_DISABLED:
                return "updates of the IronSide SE Recovery firmware are disabled on the device"
            case _:
                return "unrecognized update status"

    @classmethod
    def decode(cls, status_bytes) -> UpdateStatus:
        status = int.from_bytes(status_bytes, "little")
        return cls(status)


class FirmwareSlot(str, Enum):
    USLOT = "uslot"
    RSLOT = "rslot"

    @property
    def description(self) -> str:
        match self:
            case FirmwareSlot.USLOT:
                return "IronSide SE"
            case FirmwareSlot.RSLOT:
                return "IronSide SE Recovery"

    @property
    def update_hex_name(self) -> str:
        match self:
            case FirmwareSlot.USLOT:
                return "ironside_se_update.hex"
            case FirmwareSlot.RSLOT:
                return "ironside_se_recovery_update.hex"


def decode_version(version_bytes: bytes) -> str:
    seqnum, patch, minor, major = struct.unpack("bbbb", version_bytes[0:4])
    extraversion = c_char_p(version_bytes[4:]).value.decode("utf-8")
    return f"{major}.{minor}.{patch}-{extraversion}+{seqnum}"


def format_version(version_int: int, extraversion: str) -> str:
    major = (version_int >> 24) & 0xFF
    minor = (version_int >> 16) & 0xFF
    patch = (version_int >> 8) & 0xFF
    seqnum = version_int & 0xFF
    return f"{major}.{minor}.{patch}-{extraversion}+{seqnum}"


def _load_update_version(update_file: Path) -> str:
    ihex = IntelHex(str(update_file))
    start_addr = ihex.minaddr()
    if start_addr is None:
        raise ValueError(f"{update_file} is empty")
    version_addr = start_addr + MANIFEST_VERSION_OFFSET
    version_bytes = bytes(
        [ihex[a] for a in range(version_addr, version_addr + IRONSIDE_VERSION_LEN)]
    )
    return decode_version(version_bytes)


class UpdateBackend(abc.ABC):
    @abc.abstractmethod
    def prepare(self, update_dir: Path) -> None: ...

    @abc.abstractmethod
    def read_versions(self) -> dict[FirmwareSlot, str]: ...

    @abc.abstractmethod
    def read_update_status(self) -> UpdateStatus: ...

    @abc.abstractmethod
    def program_update(self, slot: FirmwareSlot, update_file: Path) -> None: ...

    @abc.abstractmethod
    def reset_and_wait(self) -> None: ...

    @abc.abstractmethod
    def cleanup(self) -> None: ...

    @abc.abstractmethod
    def close(self) -> None: ...


IRONSIDE_SE_VERSION_ADDR = 0x2F88_FD04
IRONSIDE_SE_RECOVERY_VERSION_ADDR = 0x2F88_FD14

MANIFEST_VERSION_OFFSET = 0x60
IRONSIDE_VERSION_LEN = 16

UPDATE_STATUS_ADDR = 0x2F88_FD24


class NrfutilBackend(UpdateBackend):
    def __init__(self, cmd: WestCommand, serial_number: str | None, wait_time: float) -> None:
        self._cmd = cmd
        self._serial_number = serial_number
        self._wait_time = wait_time
        self._sdfw_variant = None
        if self._nrfutil_supports_sdfw_variant():
            self._sdfw_variant = "ironside"

    def prepare(self, update_dir: Path) -> None:
        update_app = update_dir / "update_application.hex"
        if not update_app.exists():
            self._cmd.die(
                f"Update application file not found in ZIP: {update_app.relative_to(update_dir)}"
            )

        self._cmd.inf("Erasing non-volatile memory")
        self._nrfutil_device("recover")

        self._cmd.dbg("Programming application firmware used to trigger the update")
        self._program(update_app, erase=True)

    def read_versions(self) -> dict[FirmwareSlot, str]:
        return {
            FirmwareSlot.USLOT: self._read_firmware_version(FirmwareSlot.USLOT),
            FirmwareSlot.RSLOT: self._read_firmware_version(FirmwareSlot.RSLOT),
        }

    def read_update_status(self) -> UpdateStatus:
        raw = self._nrfutil_read(UPDATE_STATUS_ADDR, 4)
        try:
            return UpdateStatus.decode(raw)
        except ValueError:
            status = int.from_bytes(raw, "little")
            self._cmd.die(f"Read unrecognized update status from the device: 0x{status:09_X}")

    def program_update(self, slot: FirmwareSlot, update_file: Path) -> None:
        update_status = self.read_update_status()
        self._cmd.dbg(f"Status before triggering the update: {update_status}")

        self._program(update_file)

    def reset_and_wait(self) -> None:
        self._cmd.dbg("Reset to execute update service")
        self._nrfutil_device("reset")
        self._wait_for_bootstatus()
        time.sleep(0.200)

        self._cmd.dbg("Reset to trigger update installation")
        self._nrfutil_device(
            "reset --reset-kind RESET_VIA_SECDOM",
            die_on_error=False,
        )
        self._cmd.dbg("Waiting for update to complete")
        self._wait_for_bootstatus()

    def cleanup(self) -> None:
        self._cmd.dbg("Erasing application firmware used to trigger the update")
        self._nrfutil_device("erase --all")

    def close(self) -> None:
        pass

    def _program(self, hex_file: Path, erase: bool = False) -> None:
        if not hex_file.exists():
            self._cmd.die(f"Firmware file does not exist: {hex_file}")

        options = " --options chip_erase_mode=ERASE_NONE" if not erase else ""
        self._nrfutil_device(f"program{options} --firmware {hex_file}")

    def _nrfutil_device(
        self,
        cmd: str,
        die_on_error: bool = True,
        dbg_log_stdout: bool = True,
    ) -> str:
        optional_args = ""
        if self._serial_number is not None:
            optional_args += f" --serial-number {self._serial_number}"
        if self._sdfw_variant is not None:
            optional_args += f"  --x-sdfw-variant {self._sdfw_variant}"

        cmd = f"nrfutil device {cmd}{optional_args}"
        self._cmd.dbg(cmd)

        result = subprocess.run(cmd, shell=True, text=True, capture_output=True)

        if dbg_log_stdout:
            self._cmd.dbg(result.stdout)

        if result.returncode != 0:
            if die_on_error:
                self._cmd.die(f"{cmd} returned '{result.returncode}' and '{result.stderr.strip()}'")
            else:
                return ""

        return result.stdout

    def _nrfutil_read(self, address: int, num_bytes: int) -> bytes:
        json_out = json.loads(
            self._nrfutil_device(
                f"x-read --direct --address 0x{address:x} --bytes {num_bytes} "
                "--json --skip-overhead",
            )
        )
        return bytes(json_out["devices"][0]["memoryData"][0]["values"])

    def _nrfutil_supports_sdfw_variant(self) -> bool:
        nrfutil_device_program_helptext = self._nrfutil_device(
            "program --help",
            dbg_log_stdout=False,
        )
        return "--x-sdfw-variant" in nrfutil_device_program_helptext

    def _wait_for_bootstatus(self) -> int:
        boot_status = None
        start = time.perf_counter()
        while not boot_status:
            output_raw = self._nrfutil_device(
                "x-boot-status-get --json --skip-overhead",
                die_on_error=False,
            )
            if not output_raw:
                continue

            output_json = json.loads(output_raw)
            boot_status = output_json["devices"][0]["boot_status"]
            if (time.perf_counter() - start) >= self._wait_time:
                break

        if not boot_status:
            self._cmd.die("Timed out waiting for a non-zero bootstatus")

        return boot_status

    def _read_firmware_version(self, slot: FirmwareSlot) -> str:
        address = (
            IRONSIDE_SE_RECOVERY_VERSION_ADDR
            if slot == FirmwareSlot.RSLOT
            else IRONSIDE_SE_VERSION_ADDR
        )
        return decode_version(self._nrfutil_read(address, 16))


SMP_HDR_PKT = 0x0609
SMP_HDR_FRAG = 0x0414
SMP_MAX_FRAME = 127

SMP_OP_READ = 0
SMP_OP_WRITE = 2
SMP_GROUP_OS = 0
SMP_GROUP_IRONSIDE_SE = 64 + 2  # MGMT_GROUP_ID_PERUSER + 2
SMP_CMD_UPLOAD = 0
SMP_CMD_VERSION_GET = 1
SMP_CMD_STATUS_GET = 2
SMP_CMD_OS_RESET = 5

SMP_REBOOT_TIMEOUT_S = 30
SMP_REBOOT_PROBE_INTERVAL_S = 1
SMP_UPLOAD_RETRIES = 5
SMP_UPLOAD_RESPONSE_TIMEOUT_S = 1.0

SMP_WRITE_ALIGNMENT = 16


class SmpBackend(UpdateBackend):
    def __init__(self, cmd: WestCommand, port: str, baud: int, chunk_size: int) -> None:
        self._cmd = cmd
        self._chunk_size = chunk_size
        self._port = serial.Serial(port, baud, timeout=1)

    def prepare(self, update_dir: Path) -> None:
        pass

    def read_versions(self) -> dict[FirmwareSlot, str]:
        resp = smp_request(self._port, SMP_OP_READ, SMP_GROUP_IRONSIDE_SE, SMP_CMD_VERSION_GET)
        result = {}
        for slot in FirmwareSlot:
            data = resp.get(slot.value)
            if data:
                result[slot] = format_version(data["version_int"], data.get("extraversion", ""))
        return result

    def read_update_status(self) -> UpdateStatus:
        resp = smp_request(self._port, SMP_OP_READ, SMP_GROUP_IRONSIDE_SE, SMP_CMD_STATUS_GET)
        code = resp.get("status", 0xFFFF_FFFF)
        try:
            return UpdateStatus(code)
        except ValueError:
            self._cmd.wrn(f"Unknown update status code: 0x{code:08X}")
            return UpdateStatus.NONE

    def program_update(self, slot: FirmwareSlot, update_file: Path) -> None:
        ihex = IntelHex(str(update_file))
        blob = smp_pad_to_alignment(bytes(ihex.tobinarray()))
        self._upload(blob)

    def reset_and_wait(self) -> None:
        payload = cbor2.dumps({})
        pkt = smp_hdr(SMP_OP_WRITE, SMP_GROUP_OS, 0, SMP_CMD_OS_RESET, len(payload))
        pkt += payload
        for frame in smp_frame_packet(pkt):
            self._port.write(frame)
        self._port.flush()
        self._cmd.dbg("Reset device")

        self._port.close()
        deadline = time.time() + SMP_REBOOT_TIMEOUT_S

        while time.time() < deadline:
            time.sleep(SMP_REBOOT_PROBE_INTERVAL_S)

            if not self._port.is_open:
                try:
                    self._port.open()
                except serial.SerialException:
                    self._cmd.dbg("Serial port not available yet")
                    continue

            try:
                self._port.reset_input_buffer()
                smp_request(
                    self._port,
                    SMP_OP_READ,
                    SMP_GROUP_IRONSIDE_SE,
                    SMP_CMD_VERSION_GET,
                )
                return
            except (TimeoutError, ValueError):
                self._cmd.dbg("Device not ready yet")
            except serial.SerialException:
                self._cmd.dbg("Serial port lost, device still rebooting")
                if self._port.is_open:
                    self._port.close()

        self._cmd.die(f"Device did not respond within {SMP_REBOOT_TIMEOUT_S}s after reset")

    def cleanup(self) -> None:
        pass

    def close(self) -> None:
        self._port.close()

    def _upload(self, blob: bytes) -> None:
        total = len(blob)
        offset = 0
        seq = 0
        t0 = time.time()

        print(f"\r  {0:3d}%  ({offset}/{total} bytes)", end="", flush=True)

        while offset < total:
            end = min(offset + self._chunk_size, total)
            cbor_map: dict = {"off": offset, "data": blob[offset:end]}
            if offset == 0:
                cbor_map["len"] = total
            payload = cbor2.dumps(cbor_map)

            pkt = smp_hdr(
                SMP_OP_WRITE,
                SMP_GROUP_IRONSIDE_SE,
                seq,
                SMP_CMD_UPLOAD,
                len(payload),
            )
            pkt += payload

            resp = None
            for attempt in range(1, SMP_UPLOAD_RETRIES + 1):
                self._port.reset_input_buffer()
                for frame in smp_frame_packet(pkt):
                    self._port.write(frame)
                    self._port.flush()

                try:
                    resp = smp_read_response(self._port, timeout=SMP_UPLOAD_RESPONSE_TIMEOUT_S)
                    break
                except TimeoutError:
                    if attempt < SMP_UPLOAD_RETRIES:
                        # dbg since this happens pretty often
                        self._cmd.dbg(
                            f"Timeout at offset {offset}, retry {attempt}/{SMP_UPLOAD_RETRIES}"
                        )

            if resp is None:
                self._cmd.die(
                    f"Upload failed: no response at offset {offset} "
                    f"after {SMP_UPLOAD_RETRIES} attempts"
                )

            rc = resp.get("rc", -1)
            if rc != 0:
                self._cmd.die(f"Device returned error rc={rc} at offset {offset}")

            offset = resp.get("off", end)
            seq += 1
            pct = min(100, offset * 100 // total)
            print(f"\r  {pct:3d}%  ({offset}/{total} bytes)", end="", flush=True)

        elapsed = time.time() - t0
        rate = total / 1024 / elapsed if elapsed else 0
        print()
        self._cmd.inf(f"Done in {elapsed:.1f}s ({rate:.1f} KiB/s)")


def smp_crc16(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def smp_hdr(op: int, group: int, seq: int, cmd: int, payload_len: int) -> bytes:
    return struct.pack("!BBHHBB", op & 0x07, 0, payload_len, group, seq & 0xFF, cmd)


def smp_frame_packet(raw: bytes) -> list[bytes]:
    crc = smp_crc16(raw)
    data = raw + struct.pack("!H", crc)
    total = len(data)
    off = 0
    first = True
    frames: list[bytes] = []

    while off < total:
        marker = struct.pack("!H", SMP_HDR_PKT if first else SMP_HDR_FRAG)
        max_b64 = SMP_MAX_FRAME - 3
        max_raw = (max_b64 // 4) * 3

        if first:
            length_pfx = struct.pack("!H", total)
            chunk_sz = min(max_raw - 2, total - off)
            raw_payload = length_pfx + data[off : off + chunk_sz]
            first = False
        else:
            chunk_sz = min(max_raw, total - off)
            raw_payload = data[off : off + chunk_sz]

        off += chunk_sz
        frames.append(marker + base64.b64encode(raw_payload) + b"\n")

    return frames


def smp_read_response(port: serial.Serial, timeout: float = 5.0) -> dict:
    raw_frames = b""
    pkt_len = None
    deadline = time.time() + timeout
    buf = b""

    while time.time() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk

        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if len(line) < 2:
                continue
            marker = (line[0] << 8) | line[1]
            if marker == SMP_HDR_PKT:
                try:
                    decoded = base64.b64decode(line[2:])
                except Exception:
                    continue
                if len(decoded) < 2:
                    continue
                pkt_len = struct.unpack("!H", decoded[:2])[0]
                raw_frames = decoded[2:]
            elif marker == SMP_HDR_FRAG and pkt_len is not None:
                try:
                    raw_frames += base64.b64decode(line[2:])
                except Exception:
                    continue

            if pkt_len is not None and len(raw_frames) >= pkt_len:
                if smp_crc16(raw_frames[:pkt_len]) != 0:
                    pkt_len = None
                    raw_frames = b""
                    continue
                body = raw_frames[: pkt_len - 2]
                return cbor2.loads(body[8:])

    raise TimeoutError("No SMP response received")


def smp_request(
    port: serial.Serial,
    op: int,
    group: int,
    cmd: int,
    payload: dict | None = None,
    timeout: float = 5.0,
) -> dict:
    cbor_payload = cbor2.dumps(payload or {})
    pkt = smp_hdr(op, group, 0, cmd, len(cbor_payload)) + cbor_payload
    for frame in smp_frame_packet(pkt):
        port.write(frame)
    port.flush()
    return smp_read_response(port, timeout=timeout)


def smp_pad_to_alignment(blob: bytes) -> bytes:
    remainder = len(blob) % SMP_WRITE_ALIGNMENT
    if remainder:
        blob += b"\xff" * (SMP_WRITE_ALIGNMENT - remainder)
    return blob
