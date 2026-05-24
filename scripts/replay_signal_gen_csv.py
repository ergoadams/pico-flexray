#!/usr/bin/env python3
import argparse
import csv
import sys
import time
from pathlib import Path

import usb.core
import usb.util


VID = 0x3801
PID = 0xDDCC
INTERFACE = 0
EP_OUT = 0x03
EP_IN = 0x81

CMD_STOP = 0x06
CMD_TARGET = 0x09
CMD_SET_PINS = 0x0A
CMD_CLEAR_ALL = 0x0B
CMD_DIAG = 0x0E
CMD_SET_CYCLE_SLOT = 0x0F
CMD_SET_STATIC_SLOT_US = 0x10
CMD_RESET_CAPTURE_TIMING = 0x11
CMD_GET_CAPTURE_TIMING = 0x12
CMD_SET_CAPTURE_STREAM = 0x13
CMD_SET_TIMING_PAIR = 0x14
CMD_GET_TIMING_PAIR = 0x15
CMD_START = 0x05

RSP_OK = 0x00
RSP_ERR_INVALID = 0x01

DEFAULT_PINS = (
    (28, 27),
    (4, 5),
    (10, 9),
    (16, 22),
)


class ProtocolError(RuntimeError):
    pass


def parse_int(value: str) -> int:
    value = value.strip()
    return int(value, 16) if value.lower().startswith("0x") else int(value)


def parse_indicators(value: str) -> int:
    value = value.strip()
    if set(value) <= {"0", "1"} and len(value) <= 8:
        return int(value, 2)
    return parse_int(value)


def channel_mask_for_bus(bus: str) -> int:
    return {"fr1": 0x01, "fr2": 0x02}[bus]


def source_for_measure_bus(bus: str) -> int:
    return {"any": 0, "fr1": 1, "fr2": 2}[bus]


def claim(dev) -> None:
    try:
        if dev.is_kernel_driver_active(INTERFACE):
            dev.detach_kernel_driver(INTERFACE)
    except (NotImplementedError, usb.core.USBError):
        pass
    usb.util.claim_interface(dev, INTERFACE)


def read_response(dev, name: str, timeout_ms: int) -> bytes:
    deadline = time.monotonic() + (timeout_ms / 1000.0)
    last = b""
    while time.monotonic() < deadline:
        try:
            rsp = bytes(dev.read(EP_IN, 64, timeout=timeout_ms))
        except usb.core.USBError as exc:
            if getattr(exc, "errno", None) is None:
                raise
            last = bytes(str(exc), "ascii", errors="ignore")
            continue
        if rsp and rsp[0] in (RSP_OK, RSP_ERR_INVALID, 0x03):
            return rsp
        last = rsp
    raise ProtocolError(f"{name} timed out waiting for command response; last={last.hex(' ')}")


def drain_in(dev, timeout_ms: int = 25, max_reads: int = 64) -> None:
    for _ in range(max_reads):
        try:
            dev.read(EP_IN, 512, timeout=timeout_ms)
        except usb.core.USBError:
            break


def xfer(dev, name: str, packet: bytes, timeout_ms: int = 2000) -> bytes:
    dev.write(EP_OUT, packet, timeout=timeout_ms)
    rsp = read_response(dev, name, timeout_ms)
    if not rsp or rsp[0] != RSP_OK:
        status = "empty" if not rsp else f"0x{rsp[0]:02x}"
        raise ProtocolError(f"{name} rejected: status={status}, rsp={rsp.hex(' ')}")
    return rsp


def build_pin_packet(pins: tuple[tuple[int, int], ...]) -> bytes:
    data = bytearray([CMD_SET_PINS])
    for tx, txen in pins:
        data.extend((tx, txen))
    return bytes(data)


def build_cycle_slot_packet(slot: int, channel_mask: int, frame_id: int, indicators: int,
                            cycle_count: int, payload: bytes) -> bytes:
    return bytes([
        CMD_SET_CYCLE_SLOT,
        slot & 0xFF,
        channel_mask & 0x0F,
        frame_id & 0xFF,
        (frame_id >> 8) & 0xFF,
        indicators & 0x1F,
        cycle_count & 0x3F,
        len(payload) & 0xFF,
        (len(payload) >> 8) & 0xFF,
    ]) + payload


def build_static_slot_us_packet(slot_us: int) -> bytes:
    return bytes([
        CMD_SET_STATIC_SLOT_US,
        slot_us & 0xFF,
        (slot_us >> 8) & 0xFF,
        (slot_us >> 16) & 0xFF,
        (slot_us >> 24) & 0xFF,
    ])


def build_capture_stream_packet(enabled: bool) -> bytes:
    return bytes([CMD_SET_CAPTURE_STREAM, 1 if enabled else 0])


def build_timing_pair_packet(source: int, from_id: int, to_id: int) -> bytes:
    return bytes([
        CMD_SET_TIMING_PAIR,
        source & 0xFF,
        from_id & 0xFF,
        (from_id >> 8) & 0xFF,
        to_id & 0xFF,
        (to_id >> 8) & 0xFF,
    ])


def parse_capture_timing(rsp: bytes) -> dict[str, dict[str, int]]:
    if len(rsp) < 41:
        raise ProtocolError(f"capture timing response too short: {rsp.hex(' ')}")
    values = [int.from_bytes(rsp[1 + i * 4:5 + i * 4], "little") for i in range(10)]
    return {
        "fr1": {
            "last": values[0],
            "min": values[1],
            "max": values[2],
            "avg": values[3],
            "count": values[4],
        },
        "fr2": {
            "last": values[5],
            "min": values[6],
            "max": values[7],
            "avg": values[8],
            "count": values[9],
        },
    }


def format_capture_timing(timing: dict[str, dict[str, int]]) -> str:
    parts = []
    for bus in ("fr1", "fr2"):
        item = timing[bus]
        if item["count"] == 0:
            parts.append(f"{bus}: no FSS deltas yet")
        else:
            parts.append(
                f"{bus}: last={item['last']}us min={item['min']}us "
                f"max={item['max']}us avg={item['avg']}us count={item['count']}"
            )
    return " | ".join(parts)


def parse_pair_timing(rsp: bytes) -> dict[str, int]:
    if len(rsp) < 27:
        raise ProtocolError(f"pair timing response too short: {rsp.hex(' ')}")
    return {
        "enabled": rsp[1],
        "source": rsp[2],
        "from_id": int.from_bytes(rsp[3:5], "little"),
        "to_id": int.from_bytes(rsp[5:7], "little"),
        "last": int.from_bytes(rsp[7:11], "little"),
        "min": int.from_bytes(rsp[11:15], "little"),
        "max": int.from_bytes(rsp[15:19], "little"),
        "avg": int.from_bytes(rsp[19:23], "little"),
        "count": int.from_bytes(rsp[23:27], "little"),
    }


def format_pair_timing(timing: dict[str, int]) -> str:
    source_name = {0: "any", 1: "fr1", 2: "fr2"}.get(timing["source"], str(timing["source"]))
    prefix = f"{source_name} 0x{timing['from_id']:x}->0x{timing['to_id']:x}"
    if timing["count"] == 0:
        return f"{prefix}: no matching frame pair yet"
    return (
        f"{prefix}: last={timing['last']}us min={timing['min']}us "
        f"max={timing['max']}us avg={timing['avg']}us count={timing['count']}"
    )


def load_entries(path: Path, source: int, ids: set[int] | None) -> list[dict[str, int | bytes]]:
    rows: list[dict[str, int | bytes]] = []
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if parse_int(row["source"]) != source:
                continue
            frame_id = parse_int(row["frame_id"])
            if ids is not None and frame_id not in ids:
                continue
            cycle = parse_int(row["cycle_count"])
            payload = bytes.fromhex(row["payload"].strip())
            payload_words = parse_int(row["payload_length_words"])
            if payload_words * 2 != len(payload):
                raise ValueError(
                    f"frame 0x{frame_id:x} cycle {cycle}: payload_length_words={payload_words} "
                    f"but payload has {len(payload)} byte(s)"
                )
            if len(payload) > 64:
                raise ValueError(
                    f"frame 0x{frame_id:x} cycle {cycle}: payload has {len(payload)} byte(s); "
                    "cycle-specific replay currently supports up to 64 bytes"
                )
            rows.append({
                "frame_id": frame_id,
                "cycle": cycle,
                "indicators": parse_indicators(row["indicators"]),
                "payload": payload,
            })

    unique_ids = sorted({int(row["frame_id"]) for row in rows})
    if len(unique_ids) > 64:
        raise ValueError(f"{len(unique_ids)} unique id(s) selected; firmware supports 64 generator slots")
    slot_by_id = {frame_id: slot for slot, frame_id in enumerate(unique_ids)}

    entries_by_key: dict[tuple[int, int], dict[str, int | bytes]] = {}
    for row in rows:
        frame_id = int(row["frame_id"])
        cycle = int(row["cycle"])
        entries_by_key[(frame_id, cycle)] = {
            **row,
            "slot": slot_by_id[frame_id],
        }
    return [entries_by_key[key] for key in sorted(entries_by_key)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay selected CSV FlexRay frames through signal-gen firmware.")
    parser.add_argument("csv", type=Path, nargs="?", help="CSV log to replay")
    parser.add_argument("--source", type=parse_int, default=1, help="source number to select from the CSV, default 1")
    parser.add_argument("--id", dest="ids", action="append", type=parse_int,
                        help="frame ID to replay; repeatable. If omitted, all IDs from --source are used")
    parser.add_argument("--bus", choices=("fr1", "fr2"), default="fr1", help="bus to generate onto, default fr1")
    parser.add_argument("--duration", type=float,
                        help="seconds to leave replay running; omit to leave it running")
    parser.add_argument("--slot-us", type=int, default=0,
                        help="static slot FSS-to-FSS spacing in microseconds; 0 uses firmware automatic minimum")
    parser.add_argument("--measure", type=float,
                        help="seconds after START to wait, then print Pico-measured FSS-to-FSS timing")
    parser.add_argument("--measure-only", type=float,
                        help="passively measure bus FSS-to-FSS timing for this many seconds without replaying")
    parser.add_argument("--measure-pair", nargs=2, metavar=("FROM_ID", "TO_ID"), type=parse_int,
                        help="measure FSS delta only when FROM_ID is followed by TO_ID")
    parser.add_argument("--measure-bus", choices=("any", "fr1", "fr2"), default="any",
                        help="bus/source for --measure-pair, default any")
    parser.add_argument("--timeout-ms", type=int, default=2000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.slot_us < 0:
        print("--slot-us must be >= 0", file=sys.stderr)
        return 1
    if args.measure is not None and args.measure < 0:
        print("--measure must be >= 0", file=sys.stderr)
        return 1
    if args.measure_only is not None and args.measure_only < 0:
        print("--measure-only must be >= 0", file=sys.stderr)
        return 1
    ids = set(args.ids) if args.ids else None

    if args.measure_only is None and args.csv is None:
        print("CSV path is required unless --measure-only is used.", file=sys.stderr)
        return 1

    if args.measure_only is not None and args.csv is None:
        entries = []
    else:
        try:
            entries = load_entries(args.csv, args.source, ids)
        except (OSError, ValueError) as exc:
            print(f"CSV error: {exc}", file=sys.stderr)
            return 1
        if not entries:
            print("No matching CSV entries found.", file=sys.stderr)
            return 1

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("FlexRay Signal Generator USB device not found (VID 0x3801, PID 0xddcc).", file=sys.stderr)
        return 1

    try:
        if dev.get_active_configuration() is None:
            dev.set_configuration()
    except usb.core.USBError:
        dev.set_configuration()

    channel_mask = channel_mask_for_bus(args.bus)
    unique_ids = sorted({int(entry["frame_id"]) for entry in entries})

    try:
        claim(dev)
        xfer(dev, "stop", bytes([CMD_STOP]), args.timeout_ms)
        xfer(dev, "target", bytes([CMD_TARGET]), args.timeout_ms)

        if args.measure_only is not None:
            xfer(dev, "capture-stream-off", build_capture_stream_packet(False), args.timeout_ms)
            drain_in(dev)
            if args.measure_pair:
                xfer(
                    dev,
                    "set-timing-pair",
                    build_timing_pair_packet(
                        source_for_measure_bus(args.measure_bus),
                        args.measure_pair[0],
                        args.measure_pair[1],
                    ),
                    args.timeout_ms,
                )
            xfer(dev, "reset-capture-timing", bytes([CMD_RESET_CAPTURE_TIMING]), args.timeout_ms)
            time.sleep(args.measure_only)
            if args.measure_pair:
                timing = parse_pair_timing(xfer(dev, "pair-timing", bytes([CMD_GET_TIMING_PAIR]), args.timeout_ms))
                print(format_pair_timing(timing))
            else:
                timing = parse_capture_timing(xfer(dev, "capture-timing", bytes([CMD_GET_CAPTURE_TIMING]), args.timeout_ms))
                print(format_capture_timing(timing))
            xfer(dev, "capture-stream-on", build_capture_stream_packet(True), args.timeout_ms)
            return 0

        xfer(dev, "set-pins", build_pin_packet(DEFAULT_PINS), args.timeout_ms)
        xfer(dev, "clear-all", bytes([CMD_CLEAR_ALL]), args.timeout_ms)
        xfer(dev, "capture-stream-on", build_capture_stream_packet(True), args.timeout_ms)
        xfer(dev, "set-slot-us", build_static_slot_us_packet(args.slot_us), args.timeout_ms)

        for index, entry in enumerate(entries, start=1):
            packet = build_cycle_slot_packet(
                int(entry["slot"]),
                channel_mask,
                int(entry["frame_id"]),
                int(entry["indicators"]),
                int(entry["cycle"]),
                entry["payload"],  # type: ignore[arg-type]
            )
            xfer(dev, f"set-cycle-{index}", packet, args.timeout_ms)

        xfer(dev, "reset-capture-timing", bytes([CMD_RESET_CAPTURE_TIMING]), args.timeout_ms)
        if args.measure_pair:
            xfer(
                dev,
                "set-timing-pair",
                build_timing_pair_packet(
                    source_for_measure_bus(args.measure_bus),
                    args.measure_pair[0],
                    args.measure_pair[1],
                ),
                args.timeout_ms,
            )
        xfer(dev, "start", bytes([CMD_START]), args.timeout_ms)
        print(
            f"Replay started: {len(entries)} cycle-specific entrie(s), "
            f"{len(unique_ids)} id(s) from source {args.source} onto {args.bus}, "
            f"slot_us={'auto' if args.slot_us == 0 else args.slot_us}: "
            + " ".join(f"0x{frame_id:x}" for frame_id in unique_ids)
        )

        if args.measure is not None:
            time.sleep(args.measure)
            if args.measure_pair:
                timing = parse_pair_timing(xfer(dev, "pair-timing", bytes([CMD_GET_TIMING_PAIR]), args.timeout_ms))
                print(format_pair_timing(timing))
            else:
                timing = parse_capture_timing(xfer(dev, "capture-timing", bytes([CMD_GET_CAPTURE_TIMING]), args.timeout_ms))
                print(format_capture_timing(timing))

        if args.duration is not None:
            time.sleep(args.duration)
            xfer(dev, "stop", bytes([CMD_STOP]), args.timeout_ms)
            print("Replay stopped")
        return 0
    except (usb.core.USBError, ProtocolError) as exc:
        print(f"USB/protocol error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            usb.util.release_interface(dev, INTERFACE)
        except usb.core.USBError:
            pass
        usb.util.dispose_resources(dev)


if __name__ == "__main__":
    raise SystemExit(main())
