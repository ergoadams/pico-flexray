#!/usr/bin/env python3
import argparse
import csv
import struct
import sys
import time
from datetime import datetime
from typing import Iterable, Iterator, Optional


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC
EP_VENDOR_OUT = 0x03
OP_REPLAY_FRAME = 0x94
MAX_FRAME_BYTES = 262

BUS_TO_DIRECTION = {
    "1": 0,
    "fr1": 0,
    "2": 1,
    "fr2": 1,
    "3": 2,
    "fr3": 2,
    "4": 3,
    "fr4": 3,
}


def parse_int(text: str) -> int:
    return int(text.strip(), 0)


def parse_indicators(text: str) -> int:
    text = text.strip()
    if text.startswith(("0b", "0B", "0x", "0X")):
        return int(text, 0)
    if text and set(text) <= {"0", "1"}:
        return int(text, 2)
    return int(text, 10)


def parse_timestamp(text: str) -> Optional[float]:
    text = text.strip()
    if not text:
        return None
    try:
        return datetime.fromisoformat(text).timestamp()
    except ValueError:
        return None


def bus_direction(text: str) -> int:
    key = text.strip().lower()
    if key not in BUS_TO_DIRECTION:
        raise argparse.ArgumentTypeError("bus must be one of FR1, FR2, FR3, FR4")
    return BUS_TO_DIRECTION[key]


def parse_id_set(items: Optional[Iterable[str]]) -> Optional[set[int]]:
    if not items:
        return None
    ids: set[int] = set()
    for item in items:
        for part in item.split(","):
            part = part.strip()
            if part:
                frame_id = parse_int(part)
                if not 0 <= frame_id < 2048:
                    raise ValueError(f"frame id out of range: {frame_id}")
                ids.add(frame_id)
    return ids


def build_frame_bytes(row: dict[str, str], recompute_header_crc: bool = False) -> bytes:
    if recompute_header_crc:
        raise NotImplementedError("header CRC recompute is not implemented; replay uses CSV CRC fields")

    indicators = parse_indicators(row["indicators"])
    frame_id = parse_int(row["frame_id"])
    payload_length_words = parse_int(row["payload_length_words"])
    header_crc = parse_int(row["header_crc"])
    cycle_count = parse_int(row["cycle_count"])
    payload = bytes.fromhex(row["payload"].strip())
    frame_crc = parse_int(row["frame_crc"])

    expected_payload_len = payload_length_words * 2
    if len(payload) != expected_payload_len:
        raise ValueError(
            f"id {frame_id}: payload has {len(payload)} bytes, "
            f"CSV length says {expected_payload_len}"
        )
    if len(payload) > 254:
        raise ValueError(f"id {frame_id}: payload too long")

    header = bytes([
        ((indicators & 0x1F) << 3) | ((frame_id >> 8) & 0x07),
        frame_id & 0xFF,
        ((payload_length_words & 0x7F) << 1) | ((header_crc >> 10) & 0x01),
        (header_crc >> 2) & 0xFF,
        ((header_crc & 0x03) << 6) | (cycle_count & 0x3F),
    ])
    crc = bytes([
        (frame_crc >> 16) & 0xFF,
        (frame_crc >> 8) & 0xFF,
        frame_crc & 0xFF,
    ])
    frame = header + payload + crc
    if len(frame) > MAX_FRAME_BYTES:
        raise ValueError(f"id {frame_id}: frame too long")
    return frame


def iter_rows(path: str, ids: Optional[set[int]], source: Optional[str]) -> Iterator[dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {
            "timestamp",
            "source",
            "indicators",
            "frame_id",
            "payload_length_words",
            "header_crc",
            "cycle_count",
            "payload",
            "frame_crc",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(sorted(missing))}")

        for row in reader:
            if ids is not None and parse_int(row["frame_id"]) not in ids:
                continue
            if source is not None and row["source"].strip() != source:
                continue
            yield row


def find_device():
    try:
        import usb.core  # type: ignore
    except Exception:
        print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
        return None

    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        return None
    try:
        dev.set_configuration()
    except Exception:
        pass
    return dev


def send_frame(dev, direction: int, frame: bytes) -> None:
    payload = struct.pack("<BBH", OP_REPLAY_FRAME, direction, len(frame)) + frame
    dev.write(EP_VENDOR_OUT, payload, timeout=1000)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a pico-flexray CSV log to one FlexRay bus")
    parser.add_argument("csv", help="CSV log from flexray_stream_recorder.py")
    parser.add_argument("--bus", required=True, type=bus_direction, help="Output bus: FR1, FR2, FR3, or FR4")
    parser.add_argument("--source", help="Only replay rows with this CSV source value")
    parser.add_argument("--id", action="append", dest="ids", help="Only replay frame id(s), comma-separated allowed")
    parser.add_argument("--limit", type=int, help="Stop after sending this many frames")
    parser.add_argument("--loop", action="store_true", help="Replay the selected rows repeatedly")
    parser.add_argument("--no-timing", action="store_true", help="Send as fast as USB/Pico accept frames")
    parser.add_argument("--speed", type=float, default=1.0, help="Timing multiplier, e.g. 2.0 is twice as fast")
    parser.add_argument("--min-gap-ms", type=float, default=0.0, help="Minimum delay between frames")
    parser.add_argument("--dry-run", action="store_true", help="Parse and pace the log without opening USB")
    args = parser.parse_args()

    if args.speed <= 0:
        print("--speed must be greater than zero", file=sys.stderr)
        return 2
    if args.limit is not None and args.limit < 0:
        print("--limit must be zero or greater", file=sys.stderr)
        return 2

    try:
        ids = parse_id_set(args.ids)
        selected_rows = list(iter_rows(args.csv, ids, args.source))
    except Exception as e:
        print(f"CSV error: {e}", file=sys.stderr)
        return 1

    if not selected_rows:
        print("No rows matched the selected filters", file=sys.stderr)
        return 1

    dev = None
    if not args.dry_run:
        dev = find_device()
        if dev is None:
            print("Device not found. Is the Pico connected and running pico-flexray?", file=sys.stderr)
            return 1

    sent = 0
    pass_count = 0
    min_gap_s = max(0.0, args.min_gap_ms / 1000.0)

    print(
        f"{'Dry-run' if args.dry_run else 'Replaying'} {len(selected_rows)} selected row(s) "
        f"to FR{args.bus + 1}"
    )
    try:
        while True:
            pass_count += 1
            previous_ts: Optional[float] = None
            previous_send = time.monotonic()

            for row in selected_rows:
                if args.limit is not None and sent >= args.limit:
                    raise KeyboardInterrupt

                frame = build_frame_bytes(row)
                row_ts = parse_timestamp(row["timestamp"])
                delay_s = 0.0
                if not args.no_timing and previous_ts is not None and row_ts is not None:
                    delay_s = max(0.0, (row_ts - previous_ts) / args.speed)
                delay_s = max(delay_s, min_gap_s - (time.monotonic() - previous_send))
                if delay_s > 0:
                    time.sleep(delay_s)

                if dev is not None:
                    send_frame(dev, args.bus, frame)

                sent += 1
                previous_ts = row_ts
                previous_send = time.monotonic()

                if sent % 1000 == 0:
                    print(f"sent={sent} pass={pass_count}")

            if not args.loop:
                break

    except KeyboardInterrupt:
        pass

    print(f"Done. Frames {'parsed' if args.dry_run else 'queued'}: {sent}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
