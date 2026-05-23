#!/usr/bin/env python3
import argparse
import struct
import sys

try:
    import usb.core  # type: ignore
except Exception:
    print("PyUSB is required. Install with: pip install pyusb", file=sys.stderr)
    sys.exit(1)


PANDA_VID = 0x3801
PANDA_PID = 0xDDCC
EP_VENDOR_OUT = 0x03
MAX_FILTER_RULES = 96

DIR_BITS = {
    "12": 0x01,
    "21": 0x02,
    "34": 0x04,
    "43": 0x08,
}


def parse_int(value: str) -> int:
    return int(value, 0)


def parse_rule(text: str) -> tuple[int, int]:
    if ":" in text:
        id_text, dirs_text = text.split(":", 1)
    else:
        id_text, dirs_text = text, "all"

    frame_id = parse_int(id_text)
    if not 0 <= frame_id < 2048:
        raise argparse.ArgumentTypeError(f"frame id out of range: {frame_id}")

    if dirs_text == "all":
        return frame_id, 0x0F

    mask = 0
    for item in dirs_text.split(","):
        item = item.strip()
        if item not in DIR_BITS:
            raise argparse.ArgumentTypeError(f"unknown direction '{item}'")
        mask |= DIR_BITS[item]
    return frame_id, mask


def find_device():
    dev = usb.core.find(idVendor=PANDA_VID, idProduct=PANDA_PID)
    if dev is None:
        return None
    try:
        dev.set_configuration()
    except Exception:
        pass
    return dev


def main() -> int:
    parser = argparse.ArgumentParser(description="Configure pico-flexray forwarding filters")
    parser.add_argument("rules", nargs="*", type=parse_rule,
                        help="ID[:dirs], dirs are 12,21,34,43 comma-separated; default dirs=all")
    parser.add_argument("--disable", action="store_true", help="install rules but disable filtering")
    parser.add_argument("--clear", action="store_true", help="clear all forwarding filters")
    args = parser.parse_args()

    dev = find_device()
    if dev is None:
        print("Device not found", file=sys.stderr)
        return 1

    if args.clear:
        dev.write(EP_VENDOR_OUT, bytes([0x92, 0, 0, 0x93]), timeout=1000)
        print("Cleared forwarding filters")
        return 0

    if len(args.rules) > MAX_FILTER_RULES:
        print(f"At most {MAX_FILTER_RULES} filter rules are supported", file=sys.stderr)
        return 1

    payload = bytearray([0x92, 0 if args.disable else 1, len(args.rules)])
    for frame_id, direction_mask in args.rules:
        payload += struct.pack("<HB", frame_id, direction_mask)

    dev.write(EP_VENDOR_OUT, payload, timeout=1000)
    print(f"Installed {len(args.rules)} forwarding filter rule(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
