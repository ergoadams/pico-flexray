#!/usr/bin/env python3
import argparse
import csv
import struct
import sys
import time
from collections import OrderedDict

try:
    import usb.core  # type: ignore
    import usb.util  # type: ignore
except Exception:
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    sys.exit(1)


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC
INTERFACE = 0
EP_VENDOR_OUT = 0x03
EP_VENDOR_IN = 0x81
MAX_REPLAY_SLOTS = 96
MIN_BODY_LEN = 11

OP_REPLAY_SET_SLOT = 0x94
OP_REPLAY_CLEAR_SLOT = 0x95
OP_REPLAY_START = 0x96
OP_REPLAY_STOP = 0x97


def parse_int(value: str) -> int:
    return int(value.strip(), 0)


def parse_indicators(value: str) -> int:
    value = value.strip()
    if not value:
        return 0
    if set(value) <= {"0", "1"} and len(value) <= 5:
        return int(value, 2)
    return int(value, 0)


def find_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        return None
    try:
        dev.set_configuration()
    except Exception:
        pass
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except Exception:
        pass
    try:
        usb.util.claim_interface(dev, INTERFACE)
    except Exception:
        pass
    return dev


def make_set_slot(slot: int, frame: dict, cycle_specific: bool) -> bytes:
    if cycle_specific:
        cycle_base = frame["cycle_count"] & 0x3F
        cycle_mask = 0x3F
    else:
        cycle_base = 0
        cycle_mask = 0

    payload = frame["payload"]
    header = struct.pack(
        "<BBBBHBH",
        OP_REPLAY_SET_SLOT,
        slot & 0xFF,
        cycle_base,
        cycle_mask,
        frame["frame_id"] & 0x7FF,
        frame["indicators"] & 0x1F,
        len(payload),
    )
    return header + payload


def load_frames(path: str, source_filter: set[int] | None, cycle_specific: bool) -> list[dict]:
    selected: "OrderedDict[tuple[int, int] | int, dict]" = OrderedDict()
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"frame_id", "cycle_count", "payload"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"CSV is missing required columns: {', '.join(sorted(missing))}")

        for row in reader:
            if source_filter is not None:
                try:
                    source = parse_int(row.get("source", ""))
                except ValueError:
                    continue
                if source not in source_filter:
                    continue

            payload_hex = (row.get("payload") or "").strip().replace(" ", "")
            if len(payload_hex) % 2 != 0:
                continue
            payload = bytes.fromhex(payload_hex)
            if len(payload) > 254 or len(payload) & 1:
                continue

            frame_id = parse_int(row["frame_id"])
            cycle_count = parse_int(row["cycle_count"]) & 0x3F
            indicators = parse_indicators(row.get("indicators", "0"))
            key = (frame_id, cycle_count) if cycle_specific else frame_id
            if key in selected:
                continue
            selected[key] = {
                "frame_id": frame_id,
                "cycle_count": cycle_count,
                "indicators": indicators,
                "payload": payload,
            }

            if len(selected) >= MAX_REPLAY_SLOTS:
                break

    return list(selected.values())


def parse_varlen_records(buffer: bytes) -> tuple[int, list[dict]]:
    frames = []
    i = 0
    while i + 2 <= len(buffer):
        body_len = buffer[i] | (buffer[i + 1] << 8)
        if body_len < MIN_BODY_LEN:
            i += 1
            continue
        if i + 2 + body_len > len(buffer):
            break
        source = buffer[i + 2]
        header = buffer[i + 3:i + 8]
        payload_words = (header[2] >> 1) & 0x7F
        payload_len = payload_words * 2
        if 1 + 5 + payload_len + 3 != body_len:
            i += 1
            continue
        frame_id = ((header[0] & 0x07) << 8) | header[1]
        cycle_count = header[4] & 0x3F
        frames.append({"source": source, "frame_id": frame_id, "cycle_count": cycle_count})
        i += 2 + body_len
    return i, frames


def monitor(dev, seconds: float) -> None:
    if seconds <= 0:
        return
    deadline = time.monotonic() + seconds
    data_buffer = b""
    total = 0
    ids: set[int] = set()
    while time.monotonic() < deadline:
        try:
            data = bytes(dev.read(EP_VENDOR_IN, 65536, timeout=200))
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as exc:
            print(f"USB read failed: {exc}", file=sys.stderr)
            break
        data_buffer += data
        consumed, frames = parse_varlen_records(data_buffer)
        if consumed:
            data_buffer = data_buffer[consumed:]
        for frame in frames:
            total += 1
            ids.add(frame["frame_id"])
    print(f"Observed {total} frame(s), {len(ids)} unique id(s) during monitor window")


def main() -> int:
    parser = argparse.ArgumentParser(description="Load a recorder CSV into single-bus replay slots.")
    parser.add_argument("csv_file")
    parser.add_argument("--source", action="append", type=parse_int,
                        help="only replay rows with this source value; can be repeated")
    parser.add_argument("--cycle-specific", action="store_true",
                        help="treat frame_id+cycle_count as separate replay slots")
    parser.add_argument("--no-start", action="store_true", help="load slots but do not start replay")
    parser.add_argument("--monitor", type=float, default=0.0,
                        help="seconds to read back captured frames after starting")
    args = parser.parse_args()

    source_filter = set(args.source) if args.source else None
    frames = load_frames(args.csv_file, source_filter, args.cycle_specific)
    if not frames:
        print("No replayable frames found", file=sys.stderr)
        return 1

    dev = find_device()
    if dev is None:
        print("Device not found", file=sys.stderr)
        return 1

    try:
        dev.write(EP_VENDOR_OUT, bytes([OP_REPLAY_STOP]), timeout=1000)
        dev.write(EP_VENDOR_OUT, bytes([OP_REPLAY_CLEAR_SLOT, 0xFF]), timeout=1000)
        for slot, frame in enumerate(frames):
            dev.write(EP_VENDOR_OUT, make_set_slot(slot, frame, args.cycle_specific), timeout=1000)
        print(f"Loaded {len(frames)} replay slot(s)")

        if not args.no_start:
            dev.write(EP_VENDOR_OUT, bytes([OP_REPLAY_START]), timeout=1000)
            print("Replay started")
            monitor(dev, args.monitor)
        return 0
    finally:
        try:
            usb.util.release_interface(dev, INTERFACE)
        except Exception:
            pass
        usb.util.dispose_resources(dev)


if __name__ == "__main__":
    raise SystemExit(main())
