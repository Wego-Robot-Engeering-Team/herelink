#!/usr/bin/env python3

import argparse
import os
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import Iterable, List, Tuple

import serial


DEFAULT_BY_ID = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
CONTROLLER_MSGIDS = {35, 65, 69}
MSG_NAMES = {
    0: "HEARTBEAT",
    29: "SCALED_PRESSURE",
    35: "RC_CHANNELS_RAW",
    65: "RC_CHANNELS",
    69: "MANUAL_CONTROL",
    109: "RADIO_STATUS",
}


@dataclass
class MavlinkFrame:
    version: int
    offset: int
    length: int
    seq: int
    sysid: int
    compid: int
    msgid: int
    payload: bytes
    raw: bytes


def default_port() -> str:
    if os.path.exists(DEFAULT_BY_ID):
        return DEFAULT_BY_ID
    return "/dev/ttyACM0"


def hexdump(data: bytes, width: int = 16) -> str:
    lines: List[str] = []
    for start in range(0, len(data), width):
        chunk = data[start:start + width]
        hex_part = " ".join(f"{byte:02x}" for byte in chunk)
        ascii_part = "".join(chr(byte) if 32 <= byte <= 126 else "." for byte in chunk)
        lines.append(f"{start:08x}  {hex_part:<{width * 3 - 1}}  {ascii_part}")
    return "\n".join(lines)


def parse_frames(data: bytes) -> Iterable[MavlinkFrame]:
    frames, _remaining = extract_frames(data)
    return frames


def extract_frames(data: bytes) -> Tuple[List[MavlinkFrame], bytes]:
    frames: List[MavlinkFrame] = []
    index = 0
    data_len = len(data)
    while index < data_len:
        magic = data[index]
        if magic == 0xFD:
            if index + 10 > data_len:
                break
            payload_len = data[index + 1]
            incompat_flags = data[index + 2]
            signature_len = 13 if incompat_flags & 0x01 else 0
            frame_len = 10 + payload_len + 2 + signature_len
            if index + frame_len > data_len:
                break
            raw = data[index:index + frame_len]
            frames.append(
                MavlinkFrame(
                    version=2,
                    offset=index,
                    length=frame_len,
                    seq=data[index + 4],
                    sysid=data[index + 5],
                    compid=data[index + 6],
                    msgid=data[index + 7]
                    | (data[index + 8] << 8)
                    | (data[index + 9] << 16),
                    payload=raw[10:10 + payload_len],
                    raw=raw,
                )
            )
            index += frame_len
            continue
        if magic == 0xFE:
            if index + 6 > data_len:
                break
            payload_len = data[index + 1]
            frame_len = 6 + payload_len + 2
            if index + frame_len > data_len:
                break
            raw = data[index:index + frame_len]
            frames.append(
                MavlinkFrame(
                    version=1,
                    offset=index,
                    length=frame_len,
                    seq=data[index + 2],
                    sysid=data[index + 3],
                    compid=data[index + 4],
                    msgid=data[index + 5],
                    payload=raw[6:6 + payload_len],
                    raw=raw,
                )
            )
            index += frame_len
            continue
        index += 1
    return frames, data[index:]


def describe_heartbeat(payload: bytes) -> str:
    if len(payload) < 9:
        return "HEARTBEAT payload too short"
    custom_mode = struct.unpack_from("<I", payload, 0)[0]
    mav_type = payload[4]
    autopilot = payload[5]
    base_mode = payload[6]
    system_status = payload[7]
    mavlink_version = payload[8]
    return (
        "HEARTBEAT "
        f"custom_mode={custom_mode} "
        f"type={mav_type} "
        f"autopilot={autopilot} "
        f"base_mode=0x{base_mode:02x} "
        f"system_status={system_status} "
        f"mavlink_version={mavlink_version}"
    )


def rc_value_to_centered(value: int) -> str:
    if value in (0, 0xFFFF):
        return ""
    if 900 <= value <= 2100:
        normalized = max(-1.0, min(1.0, (value - 1500) / 500.0))
        return f"({normalized:+.2f})"
    return ""


def format_channels(channels: List[int]) -> str:
    parts = []
    for index, value in enumerate(channels, start=1):
        if value == 0xFFFF:
            continue
        centered = rc_value_to_centered(value)
        suffix = f"{centered}" if centered else ""
        parts.append(f"ch{index}={value}{suffix}")
    return " ".join(parts) if parts else "no valid channels"


def describe_rc_channels(payload: bytes) -> str:
    if len(payload) < 42:
        return "RC_CHANNELS payload too short"
    time_boot_ms = struct.unpack_from("<I", payload, 0)[0]
    channels = list(struct.unpack_from("<18H", payload, 4))
    chancount = payload[40]
    rssi = payload[41]
    return (
        f"RC_CHANNELS time_boot_ms={time_boot_ms} chancount={chancount} "
        f"rssi={rssi} {format_channels(channels[:chancount or 18])}"
    )


def describe_rc_channels_raw(payload: bytes) -> str:
    if len(payload) < 22:
        return "RC_CHANNELS_RAW payload too short"
    time_boot_ms = struct.unpack_from("<I", payload, 0)[0]
    channels = list(struct.unpack_from("<8H", payload, 4))
    port = payload[20]
    rssi = payload[21]
    return (
        f"RC_CHANNELS_RAW time_boot_ms={time_boot_ms} port={port} "
        f"rssi={rssi} {format_channels(channels)}"
    )


def button_list(mask: int, base_index: int = 1) -> str:
    pressed = [str(base_index + bit) for bit in range(16) if mask & (1 << bit)]
    return ",".join(pressed) if pressed else "-"


def describe_manual_control(payload: bytes) -> str:
    if len(payload) < 11:
        return "MANUAL_CONTROL payload too short"
    x, y, z, r, buttons = struct.unpack_from("<hhhhH", payload, 0)
    target = payload[10]
    description = f"MANUAL_CONTROL target={target} x={x} y={y} z={z} r={r} buttons={button_list(buttons)}"
    if len(payload) >= 13:
        buttons2 = struct.unpack_from("<H", payload, 11)[0]
        description += f" buttons2={button_list(buttons2, base_index=17)}"
    if len(payload) >= 14:
        enabled_extensions = payload[13]
        description += f" ext_mask=0x{enabled_extensions:02x}"
    return description


def byte_at(payload: bytes, index: int, default: int = 0) -> int:
    if index >= len(payload):
        return default
    return payload[index]


def describe_scaled_pressure(payload: bytes) -> str:
    if len(payload) < 14:
        return "SCALED_PRESSURE payload too short"
    time_boot_ms, press_abs, press_diff, temperature = struct.unpack_from(
        "<Iffh", payload, 0
    )
    return (
        "SCALED_PRESSURE "
        f"time_boot_ms={time_boot_ms} "
        f"press_abs={press_abs:.2f}hPa "
        f"press_diff={press_diff:.2f}hPa "
        f"temperature={temperature / 100.0:.2f}C"
    )


def describe_radio_status(payload: bytes) -> str:
    if len(payload) < 8:
        return "RADIO_STATUS payload too short"
    rxerrors = struct.unpack_from("<H", payload, 0)[0]
    fixed = struct.unpack_from("<H", payload, 2)[0]
    rssi = byte_at(payload, 4)
    remrssi = byte_at(payload, 5)
    txbuf = byte_at(payload, 6)
    noise = byte_at(payload, 7)
    remnoise = byte_at(payload, 8)
    return (
        "RADIO_STATUS "
        f"rssi={rssi} remrssi={remrssi} txbuf={txbuf}% "
        f"noise={noise} remnoise={remnoise} "
        f"rxerrors={rxerrors} fixed={fixed}"
    )


def describe_frame(frame: MavlinkFrame) -> str:
    if frame.msgid == 0:
        return describe_heartbeat(frame.payload)
    if frame.msgid == 29:
        return describe_scaled_pressure(frame.payload)
    if frame.msgid == 35:
        return describe_rc_channels_raw(frame.payload)
    if frame.msgid == 65:
        return describe_rc_channels(frame.payload)
    if frame.msgid == 69:
        return describe_manual_control(frame.payload)
    if frame.msgid == 109:
        return describe_radio_status(frame.payload)
    preview = frame.payload[:24]
    return f"payload preview: {preview.hex(' ')}"


def message_label(msgid: int) -> str:
    return MSG_NAMES.get(msgid, f"MSG_{msgid}")


def configure_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read a UART port and summarize MAVLink traffic."
    )
    parser.add_argument(
        "--port",
        default=default_port(),
        help="Serial device path. Default prefers the CP2102 by-id path.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
        help="UART baud rate. Default: 921600",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=5.0,
        help="How long to capture traffic. Default: 5",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=4096,
        help="Serial read chunk size. Default: 4096",
    )
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="Print a hexdump of all received bytes.",
    )
    parser.add_argument(
        "--frame-limit",
        type=int,
        default=20,
        help="Maximum number of decoded frames to print. Default: 20",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="Only print message counts and decoded RC/manual-control frames.",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Continuously print decoded MAVLink frames until Ctrl-C.",
    )
    parser.add_argument(
        "--rc-only",
        action="store_true",
        help="In live mode, only print RC_CHANNELS, RC_CHANNELS_RAW, and MANUAL_CONTROL.",
    )
    parser.add_argument(
        "--changed-only",
        action="store_true",
        help="In live mode, only print controller messages when their payload changes.",
    )
    return parser


def read_serial(port: str, baud: int, seconds: float, chunk_size: int) -> bytes:
    deadline = time.monotonic() + seconds
    chunks: List[bytes] = []
    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        while time.monotonic() < deadline:
            try:
                chunk = ser.read(chunk_size)
            except serial.SerialException as exc:
                if "returned no data" not in str(exc):
                    raise
                time.sleep(0.05)
                continue
            if chunk:
                chunks.append(chunk)
    return b"".join(chunks)


def monitor_serial(args: argparse.Namespace) -> int:
    buffer = b""
    last_payload_by_msgid = {}
    counts = Counter()
    last_status = time.monotonic()

    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print("Live monitor started. Move sticks/switches, press Ctrl-C to stop.")

    try:
        with serial.Serial(port=args.port, baudrate=args.baud, timeout=0.2) as ser:
            while True:
                try:
                    chunk = ser.read(args.chunk_size)
                except serial.SerialException as exc:
                    if "returned no data" not in str(exc):
                        raise
                    time.sleep(0.05)
                    continue
                if chunk:
                    buffer += chunk
                    frames, buffer = extract_frames(buffer)
                    for frame in frames:
                        counts[frame.msgid] += 1
                        if args.rc_only and frame.msgid not in CONTROLLER_MSGIDS:
                            continue
                        if args.changed_only and frame.msgid in CONTROLLER_MSGIDS:
                            previous = last_payload_by_msgid.get(frame.msgid)
                            if previous == frame.payload:
                                continue
                            last_payload_by_msgid[frame.msgid] = frame.payload
                        now = time.strftime("%H:%M:%S")
                        print(
                            f"{now} {message_label(frame.msgid)} "
                            f"MAVLink{frame.version} msgid={frame.msgid} "
                            f"seq={frame.seq} sys={frame.sysid} comp={frame.compid} "
                            f"{describe_frame(frame)}",
                            flush=True,
                        )

                if time.monotonic() - last_status >= 5.0:
                    last_status = time.monotonic()
                    if args.rc_only and not any(counts[msgid] for msgid in CONTROLLER_MSGIDS):
                        print(
                            "No RC_CHANNELS, RC_CHANNELS_RAW, or MANUAL_CONTROL seen yet.",
                            flush=True,
                        )
    except KeyboardInterrupt:
        print("\nStopped.")
        if counts:
            print(
                "Message counts: "
                + ", ".join(
                    f"{msgid}:{count}" for msgid, count in sorted(counts.items())
                )
            )
        return 0
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        if "Permission denied" in str(exc):
            print(
                "Hint: run with sudo or add your user to the dialout group.",
                file=sys.stderr,
            )
        return 1


def main() -> int:
    args = configure_parser().parse_args()

    if args.live:
        return monitor_serial(args)

    try:
        captured = read_serial(args.port, args.baud, args.seconds, args.chunk_size)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        if "Permission denied" in str(exc):
            print(
                "Hint: run with sudo or add your user to the dialout group.",
                file=sys.stderr,
            )
        return 1

    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Capture time: {args.seconds:.1f}s")
    print(f"Bytes received: {len(captured)}")

    if not captured:
        print("No data received.")
        return 2

    frames = list(parse_frames(captured))
    print(f"MAVLink-like frames found: {len(frames)}")
    if frames:
        counts = Counter(frame.msgid for frame in frames)
        print(
            "Message counts: "
            + ", ".join(f"{msgid}:{count}" for msgid, count in sorted(counts.items()))
        )

    if args.show_raw or not frames:
        print("\nRaw bytes:")
        print(hexdump(captured))

    display_frames = frames
    if args.summary_only:
        display_frames = [frame for frame in frames if frame.msgid in CONTROLLER_MSGIDS]
    display_frames = display_frames[:args.frame_limit]

    for frame in display_frames:
        decoded = describe_frame(frame)
        print(
            f"\n[{frame.offset:06d}] MAVLink{frame.version} "
            f"{message_label(frame.msgid)} msgid={frame.msgid} seq={frame.seq} sys={frame.sysid} "
            f"comp={frame.compid} payload={len(frame.payload)} length={frame.length}"
        )
        print(decoded)

    if args.summary_only and not display_frames:
        print(
            "\nNo RC_CHANNELS, RC_CHANNELS_RAW, or MANUAL_CONTROL frames were seen "
            "during this capture."
        )

    if not frames:
        print(
            "\nNo MAVLink frame header was recognized. "
            "That usually means the baud rate or UART format is wrong."
        )
        return 3

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
