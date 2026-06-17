#!/usr/bin/env python3

import argparse
import json
import os
import struct
import time
from dataclasses import dataclass
from typing import Dict, Iterator, Optional

import serial

try:
    from .check_mavlink_uart import MavlinkFrame, extract_frames
except ImportError:
    from check_mavlink_uart import MavlinkFrame, extract_frames


DEFAULT_USB_PORT = "/dev/ttyACM0"
DEFAULT_UART_PORT = "/dev/ttyUSB0"
MAVLINK_V1_MAGIC = 0xFE
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAV_DATA_STREAM_RC_CHANNELS = 3
MAV_CMD_SET_MESSAGE_INTERVAL = 511
RC_CHANNELS_MSG_ID = 65
RC_CHANNELS_RAW_MSG_ID = 35
HEARTBEAT_MSG_ID = 0


def default_port() -> str:
    if os.path.exists(DEFAULT_USB_PORT):
        return DEFAULT_USB_PORT
    return DEFAULT_UART_PORT


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (
            (crc >> 8)
            ^ (tmp << 8)
            ^ (tmp << 3)
            ^ ((tmp & 0xFF) >> 4)
        ) & 0xFFFF
    return crc


@dataclass
class MavlinkWriter:
    system_id: int = 255
    component_id: int = 190
    sequence: int = 0

    def build_v1(self, msg_id: int, payload: bytes, crc_extra: int) -> bytes:
        header = bytes(
            [
                MAVLINK_V1_MAGIC,
                len(payload),
                self.sequence & 0xFF,
                self.system_id & 0xFF,
                self.component_id & 0xFF,
                msg_id & 0xFF,
            ]
        )
        self.sequence = (self.sequence + 1) % 256
        crc = x25_crc(header[1:] + payload + bytes([crc_extra]))
        return header + payload + struct.pack("<H", crc)

    def heartbeat(self) -> bytes:
        payload = struct.pack(
            "<IBBBBB",
            0,
            MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID,
            0,
            MAV_STATE_ACTIVE,
            3,
        )
        return self.build_v1(HEARTBEAT_MSG_ID, payload, crc_extra=50)

    def request_data_stream(
        self,
        target_system: int,
        target_component: int,
        stream_id: int,
        rate_hz: int,
        start_stop: int = 1,
    ) -> bytes:
        payload = struct.pack(
            "<HBBBB",
            rate_hz,
            target_system,
            target_component,
            stream_id,
            start_stop,
        )
        return self.build_v1(66, payload, crc_extra=148)

    def set_message_interval(
        self,
        target_system: int,
        target_component: int,
        message_id: int,
        interval_us: int,
    ) -> bytes:
        payload = struct.pack(
            "<7fHBBB",
            float(message_id),
            float(interval_us),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MAV_CMD_SET_MESSAGE_INTERVAL,
            target_system,
            target_component,
            0,
        )
        return self.build_v1(76, payload, crc_extra=152)


def decode_rc_channels(frame: MavlinkFrame) -> Optional[Dict[str, object]]:
    if frame.msgid == RC_CHANNELS_MSG_ID:
        if len(frame.payload) < 42:
            return None
        time_boot_ms = struct.unpack_from("<I", frame.payload, 0)[0]
        channels = list(struct.unpack_from("<18H", frame.payload, 4))
        channel_count = frame.payload[40] or 18
        rssi = frame.payload[41]
        return {
            "message": "RC_CHANNELS",
            "sysid": frame.sysid,
            "compid": frame.compid,
            "time_boot_ms": time_boot_ms,
            "rssi": rssi,
            "channels": {
                f"ch{index + 1}": value
                for index, value in enumerate(channels[:channel_count])
                if value != 0xFFFF
            },
        }

    if frame.msgid == RC_CHANNELS_RAW_MSG_ID:
        if len(frame.payload) < 22:
            return None
        time_boot_ms = struct.unpack_from("<I", frame.payload, 0)[0]
        channels = list(struct.unpack_from("<8H", frame.payload, 4))
        port = frame.payload[20]
        rssi = frame.payload[21]
        return {
            "message": "RC_CHANNELS_RAW",
            "sysid": frame.sysid,
            "compid": frame.compid,
            "time_boot_ms": time_boot_ms,
            "port": port,
            "rssi": rssi,
            "channels": {
                f"ch{index + 1}": value
                for index, value in enumerate(channels)
                if value not in (0, 0xFFFF)
            },
        }

    return None


def normalize_rc_value(value: int) -> Optional[float]:
    if value in (0, 0xFFFF):
        return None
    if not 900 <= value <= 2100:
        return None
    return max(-1.0, min(1.0, (value - 1500) / 500.0))


def format_selected_channels(message: Dict[str, object], channel_numbers: list[int]) -> str:
    channels = message.get("channels", {})
    if not isinstance(channels, dict):
        channels = {}

    parts = []
    for channel_number in channel_numbers:
        name = f"ch{channel_number}"
        value = channels.get(name)
        if isinstance(value, int):
            parts.append(f"{name}={value}")
        else:
            parts.append(f"{name}=missing")

    timestamp = message.get("time_boot_ms", "-")
    rssi = message.get("rssi", "-")
    return f"t={timestamp}ms rssi={rssi} | " + " | ".join(parts)


def send_rc_requests(
    ser: serial.Serial,
    writer: MavlinkWriter,
    target_system: int,
    target_component: int,
    rate_hz: int,
) -> None:
    interval_us = int(1_000_000 / max(rate_hz, 1))
    packets = [
        writer.heartbeat(),
        writer.request_data_stream(
            target_system=target_system,
            target_component=target_component,
            stream_id=MAV_DATA_STREAM_RC_CHANNELS,
            rate_hz=rate_hz,
            start_stop=1,
        ),
        writer.set_message_interval(
            target_system=target_system,
            target_component=target_component,
            message_id=RC_CHANNELS_MSG_ID,
            interval_us=interval_us,
        ),
        writer.set_message_interval(
            target_system=target_system,
            target_component=target_component,
            message_id=RC_CHANNELS_RAW_MSG_ID,
            interval_us=interval_us,
        ),
    ]
    for packet in packets:
        ser.write(packet)
    ser.flush()


def iter_rc_messages(
    port: str,
    baud: int = 115200,
    rate_hz: int = 10,
    target_system: int = 1,
    target_component: int = 0,
    timeout: float = 10.0,
    request_interval: float = 1.0,
) -> Iterator[Dict[str, object]]:
    writer = MavlinkWriter()
    buffer = b""
    last_request = 0.0
    deadline = time.monotonic() + timeout if timeout > 0 else None

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        while True:
            now = time.monotonic()
            if now - last_request >= request_interval:
                send_rc_requests(
                    ser=ser,
                    writer=writer,
                    target_system=target_system,
                    target_component=target_component,
                    rate_hz=rate_hz,
                )
                last_request = now

            try:
                chunk = ser.read(4096)
            except serial.SerialException as exc:
                if "returned no data" not in str(exc):
                    raise
                time.sleep(0.05)
                chunk = b""
            if chunk:
                buffer += chunk
                frames, buffer = extract_frames(buffer)
                for frame in frames:
                    if frame.msgid == HEARTBEAT_MSG_ID:
                        target_system = frame.sysid
                        target_component = frame.compid
                    decoded = decode_rc_channels(frame)
                    if decoded is not None:
                        yield decoded

            if deadline is not None and time.monotonic() >= deadline:
                return


def first_rc_message(
    port: str,
    baud: int = 115200,
    rate_hz: int = 10,
    target_system: int = 1,
    target_component: int = 0,
    timeout: float = 10.0,
) -> Optional[Dict[str, object]]:
    for message in iter_rc_messages(
        port=port,
        baud=baud,
        rate_hz=rate_hz,
        target_system=target_system,
        target_component=target_component,
        timeout=timeout,
    ):
        return message
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Request RC messages from Cube over MAVLink and print them as JSON."
    )
    parser.add_argument("--port", default=default_port(), help="Serial device path.")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. USB ACM usually ignores this, but it must be set.",
    )
    parser.add_argument(
        "--rate-hz",
        type=int,
        default=10,
        help="Requested RC update rate in Hz.",
    )
    parser.add_argument(
        "--target-system",
        type=int,
        default=1,
        help="Initial MAVLink target system id.",
    )
    parser.add_argument(
        "--target-component",
        type=int,
        default=0,
        help="Initial MAVLink target component id. Default 0 broadcasts to the autopilot.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Stop after this many seconds if no RC message arrives.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Print the first RC message and exit.",
    )
    parser.add_argument(
        "--first4",
        action="store_true",
        help="Print only channels 1-4 in a compact human-readable format.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.once:
        message = first_rc_message(
            port=args.port,
            baud=args.baud,
            rate_hz=args.rate_hz,
            target_system=args.target_system,
            target_component=args.target_component,
            timeout=args.timeout,
        )
        if message is None:
            print("No RC message received.")
            return 1
        if args.first4:
            print(format_selected_channels(message, [1, 2, 3, 4]))
            return 0
        print(json.dumps(message, sort_keys=True))
        return 0

    emitted = False
    try:
        for message in iter_rc_messages(
            port=args.port,
            baud=args.baud,
            rate_hz=args.rate_hz,
            target_system=args.target_system,
            target_component=args.target_component,
            timeout=args.timeout,
        ):
            emitted = True
            if args.first4:
                print(format_selected_channels(message, [1, 2, 3, 4]), flush=True)
            else:
                print(json.dumps(message, sort_keys=True), flush=True)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}")
        return 1

    if not emitted:
        print("No RC message received.")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
