#!/usr/bin/env python3

from __future__ import annotations

import argparse
import pathlib
import sys
import zlib


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate pokemon.gba.zlib payload for firmware embedding"
    )
    parser.add_argument("input_gba", type=pathlib.Path, help="Path to source pokemon.gba")
    parser.add_argument("output_zlib", type=pathlib.Path, help="Path to output pokemon.gba.zlib")
    parser.add_argument(
        "--level",
        type=int,
        default=9,
        choices=range(0, 10),
        metavar="0-9",
        help="zlib compression level (default: 9)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.input_gba.exists():
        print(f"Input ROM not found: {args.input_gba}", file=sys.stderr)
        return 1

    raw = args.input_gba.read_bytes()
    raw_size = len(raw)

    if raw_size == 0:
        print("Input ROM is empty", file=sys.stderr)
        return 1

    if raw_size > 0xFFFFFFFF:
        print("Input ROM is too large for 32-bit size header", file=sys.stderr)
        return 1

    compressed = zlib.compress(raw, level=args.level)
    payload = raw_size.to_bytes(4, "little") + compressed

    args.output_zlib.parent.mkdir(parents=True, exist_ok=True)
    args.output_zlib.write_bytes(payload)

    ratio = (len(compressed) / raw_size) * 100.0
    print(
        f"Generated {args.output_zlib} | raw={raw_size} bytes, "
        f"zlib={len(compressed)} bytes ({ratio:.2f}%)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
