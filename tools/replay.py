#!/usr/bin/env python3
"""Replays a .gray recording through the same detection the board runs.

The recording holds raw 8-bit grayscale exactly as the sensor produced it,
so the numbers here match what the ESP32 computed live. That is the whole
reason the recording is uncompressed: JPEG would shift pixel values and
every threshold with them.

    python tools/replay.py raw_0001.gray
    python tools/replay.py raw_0001.gray --move 2.0 --calm 0.5 --min 3
    python tools/replay.py raw_0001.gray --png out/      # write the masks

Sweep the settings to find good ones before touching the firmware:

    python tools/replay.py raw_0001.gray --sweep
"""
import argparse
import pathlib
import struct
import sys

HEADER = 16
MAGIC = b"ESPG"


def read_header(f):
    head = f.read(HEADER)
    if len(head) < HEADER or head[:4] != MAGIC:
        sys.exit("not an ESPG recording (wrong magic)")
    w, h, fps, _ = struct.unpack_from("<HHHH", head, 4)
    (frames,) = struct.unpack_from("<I", head, 12)
    return w, h, fps, frames


def frames_of(path):
    """Yields each frame as a bytes object."""
    with open(path, "rb") as f:
        w, h, fps, declared = read_header(f)
        size = w * h
        yield ("header", w, h, fps, declared)
        while True:
            buf = f.read(size)
            if len(buf) < size:
                return
            yield ("frame", buf)


def count_above(a, b, thr):
    """Pixels where |a - b| > thr. Pure Python, but only ~77k per frame."""
    n = 0
    for x, y in zip(a, b):
        d = x - y
        if d < 0:
            d = -d
        if d > thr:
            n += 1
    return n


def run(path, thr, move, calm, min_s, calm_cycles, png_dir, quiet):
    it = frames_of(path)
    _, w, h, fps, declared = next(it)
    total = w * h
    if not quiet:
        print(f"{path}: {w}x{h}, {fps} fps, {declared} frames declared")

    ref = None
    prev = None
    state = "waiting"
    start_i = 0
    calm_run = 0
    events = []

    for i, item in enumerate(it):
        frame = item[1]
        if ref is None:
            ref = frame          # first frame stands in for the reference
            prev = frame
            continue

        motion = 100.0 * count_above(frame, prev, thr) / total
        prev = frame

        if state == "waiting":
            if motion >= move:
                state, start_i, calm_run = "active", i, 0
        else:
            if motion < calm:
                calm_run += 1
                if calm_run >= calm_cycles:
                    dur = (i - calm_run - start_i) / fps
                    if dur >= min_s:
                        events.append((start_i / fps, dur))
                        if not quiet:
                            print(f"  event at {start_i/fps:6.1f} s, {dur:4.1f} s long")
                    elif not quiet:
                        print(f"  ignored at {start_i/fps:6.1f} s, only {dur:4.1f} s")
                    state, calm_run = "waiting", 0
            else:
                calm_run = 0

        if png_dir:
            write_mask_png(png_dir, i, frame, ref, thr, w, h)

    return events


def write_mask_png(out_dir, index, frame, ref, thr, w, h):
    """Writes the reference mask as a 1-bit PNG, no dependencies."""
    import binascii
    import zlib

    rows = bytearray()
    for y in range(h):
        rows.append(0)                       # filter byte: none
        base = y * w
        row = bytearray(w)
        for x in range(w):
            d = frame[base + x] - ref[base + x]
            row[x] = 0 if (d if d >= 0 else -d) > thr else 255
        rows += row

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", binascii.crc32(kind + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(rows), 6))
           + chunk(b"IEND", b""))
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / f"mask_{index:05d}.png").write_bytes(png)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--thr",   type=int,   default=30,  help="per-pixel threshold, 0..255")
    ap.add_argument("--move",  type=float, default=3.0, help="%% that counts as movement")
    ap.add_argument("--calm",  type=float, default=0.8, help="%% below which it is settled")
    ap.add_argument("--min",   type=float, default=5.0, dest="min_s",
                    help="minimum seconds of movement")
    ap.add_argument("--calm-cycles", type=int, default=8)
    ap.add_argument("--png",   metavar="DIR", help="also write every mask as a PNG")
    ap.add_argument("--sweep", action="store_true",
                    help="try a grid of settings and count the events each one finds")
    a = ap.parse_args()

    if not a.sweep:
        events = run(a.file, a.thr, a.move, a.calm, a.min_s, a.calm_cycles, a.png, False)
        print(f"{len(events)} event(s)")
        return

    print(" thr  move  calm   min | events")
    print("-----------------------+-------")
    for thr in (20, 30, 45):
        for move in (1.0, 3.0, 6.0):
            for calm in (0.3, 0.8, 2.0):
                ev = run(a.file, thr, move, calm, a.min_s, a.calm_cycles, None, True)
                print(f"{thr:4d} {move:5.1f} {calm:5.1f} {a.min_s:5.1f} | {len(ev):3d}")


if __name__ == "__main__":
    main()
