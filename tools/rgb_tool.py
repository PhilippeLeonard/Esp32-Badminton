#!/usr/bin/env python3
"""Lit les enregistrements bruts /rec_NNNN.rgb de esp32_cam_simple.

    python tools/rgb_tool.py rec_0001.rgb                 # ce que contient le fichier
    python tools/rgb_tool.py rec_0001.rgb --png 0 50 100  # trois images en PNG
    python tools/rgb_tool.py rec_0001.rgb --mp4 sortie.mp4   # via ffmpeg s'il est la

Le nombre d'images vient de la TAILLE du fichier, pas de l'entete : un
enregistrement coupe par une panne reste lisible, seule la derniere image
incomplete est ignoree.
"""
import os
import struct
import sys
import zlib

HEADER = 32


def read_header(f):
    h = f.read(HEADER)
    if len(h) < HEADER:
        raise SystemExit("fichier trop court, pas d'entete")
    if h[:4] != b"ESPC":
        raise SystemExit("magie %r inattendue (ce n'est pas un .rgb de la carte)" % h[:4])
    ver, fmt, w, hgt, fps = struct.unpack_from("<HHHHH", h, 4)
    frames = struct.unpack_from("<I", h, 16)[0]
    if fmt != 0:
        raise SystemExit("format %d inconnu (0 = RGB565 attendu)" % fmt)
    return ver, w, hgt, fps, frames


def frame_count(path, w, h):
    body = os.path.getsize(path) - HEADER
    per = w * h * 2
    return body // per, body % per, per


def to_rgb888(buf, swap):
    out = bytearray(len(buf) // 2 * 3)
    o = 0
    for i in range(0, len(buf) - 1, 2):
        hb, lb = buf[i], buf[i + 1]
        if swap:
            hb, lb = lb, hb
        out[o] = hb & 0xF8
        out[o + 1] = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3)
        out[o + 2] = (lb & 0x1F) << 3
        o += 3
    return bytes(out)


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(kind, data):
        c = kind + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as o:
        o.write(png)


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    path = args[0]
    swap = "--swap" in args
    with open(path, "rb") as f:
        ver, w, h, fps, declared = read_header(f)
        n, rest, per = frame_count(path, w, h)
        print("%s : v%d, %dx%d, %d i/s" % (os.path.basename(path), ver, w, h, fps))
        print("  %d images completes (%.1f s), entete annonce %s"
              % (n, n / float(fps or 1), declared or "0 (enregistrement interrompu)"))
        if rest:
            print("  %d octets de queue : derniere image incomplete, ignoree" % rest)
        if declared and declared != n:
            print("  l'entete et la taille ne sont pas d'accord : c'est la taille qui fait foi")

        if "--png" in args:
            want = []
            for a in args[args.index("--png") + 1:]:
                if a.startswith("--"):
                    break
                want.append(int(a))
            outdir = os.path.splitext(path)[0] + "_png"
            os.makedirs(outdir, exist_ok=True)
            for i in want:
                if i >= n:
                    print("  image %d absente" % i)
                    continue
                f.seek(HEADER + i * per)
                p = os.path.join(outdir, "f%05d.png" % i)
                write_png(p, w, h, to_rgb888(f.read(per), swap))
                print("  ->", p)

        if "--mp4" in args:
            out = args[args.index("--mp4") + 1]
            import subprocess
            cmd = ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt",
                   "rgb565be" if not swap else "rgb565le",
                   "-s", "%dx%d" % (w, h), "-r", str(fps or 10),
                   "-i", "-", "-pix_fmt", "yuv420p", out]
            print("  ", " ".join(cmd))
            f.seek(HEADER)
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
            left = n
            while left:
                p.stdin.write(f.read(per))
                left -= 1
            p.stdin.close()
            p.wait()


if __name__ == "__main__":
    main()
