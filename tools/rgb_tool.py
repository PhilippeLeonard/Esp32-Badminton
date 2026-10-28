#!/usr/bin/env python3
"""Lit les enregistrements bruts /rec_NNNN.rgb de esp32_cam_simple.

    python tools/rgb_tool.py rec_0001.rgb                 # ce que contient le fichier
    python tools/rgb_tool.py rec_0001.rgb --avi video.avi # lisible par VLC, sans rien installer
    python tools/rgb_tool.py rec_0001.rgb --png 0 50 100  # trois images en PNG
    python tools/rgb_tool.py rec_0001.rgb --mp4 sortie.mp4   # via ffmpeg s'il est la

Ajouter --flip si l'image est a l'envers.

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
    if fmt not in (0, 1):
        raise SystemExit("format %d inconnu (0 = RGB565, 1 = JPEG)" % fmt)
    return ver, fmt, w, hgt, fps, frames


def frame_count(path, w, h):
    body = os.path.getsize(path) - HEADER
    per = w * h * 2
    return body // per, body % per, per


def jpeg_index(f):
    """[(offset, longueur)] des images JPEG, chacune precedee de sa
    longueur sur 4 octets. Une derniere image tronquee par une coupure est
    ignoree : c'est tout l'interet du prefixe."""
    f.seek(0, os.SEEK_END)
    end = f.tell()
    f.seek(HEADER)
    out, pos = [], HEADER
    while pos + 4 <= end:
        f.seek(pos)
        (ln,) = struct.unpack("<I", f.read(4))
        if ln == 0 or ln > 4 * 1024 * 1024 or pos + 4 + ln > end:
            break                      # queue incomplete
        out.append((pos + 4, ln))
        pos += 4 + ln
    return out, end - pos


def write_mjpg_avi(out, f, idx, w, h, fps):
    """AVI MJPEG : les images sont deja du JPEG, on les recopie telles
    quelles. L'index est bati ici, sur le PC, ou une coupure de courant
    n'est plus un probleme."""
    movi = sum(8 + ln + (ln & 1) for _, ln in idx)
    n = len(idx)
    biggest = max((ln for _, ln in idx), default=0)

    def tag(s, v):
        return s + struct.pack('<I', v)

    hdr = bytearray(224)
    def put(off, data):
        hdr[off:off + len(data)] = data
    put(0, tag(b'RIFF', 216 + movi + 8 + 16 * n)); put(8, b'AVI ')
    put(12, tag(b'LIST', 192)); put(20, b'hdrl')
    put(24, tag(b'avih', 56))
    put(32, struct.pack('<I', 1000000 // (fps or 1)))
    put(36, struct.pack('<I', biggest * (fps or 1)))
    put(44, struct.pack('<I', 0x10))
    put(48, struct.pack('<I', n))
    put(56, struct.pack('<I', 1))
    put(60, struct.pack('<I', biggest))
    put(64, struct.pack('<II', w, h))
    put(88, tag(b'LIST', 116)); put(96, b'strl')
    put(100, tag(b'strh', 56))
    put(108, b'vids'); put(112, b'MJPG')
    put(128, struct.pack('<I', 1))
    put(132, struct.pack('<I', fps or 1))
    put(140, struct.pack('<I', n))
    put(144, struct.pack('<I', biggest))
    put(148, struct.pack('<I', 0xFFFFFFFF))
    put(156, struct.pack('<HHHH', 0, 0, w, h))
    put(164, tag(b'strf', 40))
    put(172, struct.pack('<IiiHH', 40, w, h, 1, 24))
    put(188, b'MJPG'); put(192, struct.pack('<I', w * h * 3))
    put(212, tag(b'LIST', 4 + movi)); put(220, b'movi')

    o = open(out, 'wb')
    o.write(bytes(hdr))
    index = bytearray()
    off = 4
    for pos, ln in idx:
        f.seek(pos)
        data = f.read(ln)
        o.write(b'00dc' + struct.pack('<I', ln))
        o.write(data)
        if ln & 1:
            o.write(bytes(1))            # les morceaux RIFF sont alignes
        index += b'00dc' + struct.pack('<III', 0x10, off, ln)
        off += 8 + ln + (ln & 1)
    o.write(b'idx1' + struct.pack('<I', 16 * n))
    o.write(bytes(index))
    o.close()
    print("  -> %s : %d images, %dx%d, %d i/s" % (out, n, w, h, fps))


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


def write_avi(out, src, n, w, h, fps, per, swap, flip):
    """AVI non compresse (BI_RGB 24 bits). Aucun codec a installer : VLC,
    Windows Media Player et ffmpeg le lisent tel quel. Les lignes d'un DIB
    vont du bas vers le haut et les octets sont en BGR."""
    row = w * 3
    assert row % 4 == 0, "largeur non alignee, ajouter un bourrage"
    frame_bytes = row * h
    movi = n * (8 + frame_bytes)
    idx = 16 * n

    def tag(s, v):
        return s + struct.pack('<I', v)

    hdr = bytearray(224)
    def put(off, data):
        hdr[off:off + len(data)] = data
    put(0, tag(b'RIFF', 216 + movi + 8 + idx)); put(8, b'AVI ')
    put(12, tag(b'LIST', 192)); put(20, b'hdrl')
    put(24, tag(b'avih', 56))
    put(32, struct.pack('<I', 1000000 // (fps or 1)))
    put(36, struct.pack('<I', frame_bytes * (fps or 1)))
    put(44, struct.pack('<I', 0x10))            # AVIF_HASINDEX
    put(48, struct.pack('<I', n))
    put(56, struct.pack('<I', 1))
    put(60, struct.pack('<I', frame_bytes))
    put(64, struct.pack('<II', w, h))
    put(88, tag(b'LIST', 116)); put(96, b'strl')
    put(100, tag(b'strh', 56))
    put(108, b'vids'); put(112, b'DIB ')
    put(128, struct.pack('<I', 1))              # dwScale
    put(132, struct.pack('<I', fps or 1))       # dwRate
    put(140, struct.pack('<I', n))              # dwLength
    put(144, struct.pack('<I', frame_bytes))
    put(148, struct.pack('<I', 0xFFFFFFFF))
    put(156, struct.pack('<HHHH', 0, 0, w, h))
    put(164, tag(b'strf', 40))
    put(172, struct.pack('<IiiHH', 40, w, h, 1, 24))
    put(188, struct.pack('<II', 0, frame_bytes))  # BI_RGB, taille image
    put(212, tag(b'LIST', 4 + movi)); put(220, b'movi')

    o = open(out, 'wb')
    o.write(bytes(hdr))
    index = bytearray()
    off = 4
    for i in range(n):
        src.seek(HEADER + i * per)
        rgb = to_rgb888(src.read(per), swap)
        rows = [rgb[y * row:(y + 1) * row] for y in range(h)]
        if not flip:
            rows.reverse()            # un DIB se lit de bas en haut
        bgr = b''.join(bytes(r[k:k+3][::-1]) for r in rows for k in range(0, row, 3))
        o.write(b'00db' + struct.pack('<I', frame_bytes))
        o.write(bgr)
        index += b'00db' + struct.pack('<III', 0x10, off, frame_bytes)
        off += 8 + frame_bytes
    o.write(b'idx1' + struct.pack('<I', idx))
    o.write(bytes(index))
    o.close()
    print("  -> %s : %d images, %dx%d, %d i/s" % (out, n, w, h, fps))


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit(__doc__)
    path = args[0]
    swap = "--swap" in args
    flip = "--flip" in args
    with open(path, "rb") as f:
        ver, fmt, w, h, fps, declared = read_header(f)

        if fmt == 1:                   # JPEG prefixe de sa longueur
            idx, rest = jpeg_index(f)
            n = len(idx)
            print("%s : v%d, %dx%d, %d i/s, JPEG"
                  % (os.path.basename(path), ver, w, h, fps))
            print("  %d images (%.1f s), %.2f Mo"
                  % (n, n / float(fps or 1),
                     sum(ln for _, ln in idx) / 1048576.0))
            if rest:
                print("  %d octets de queue : derniere image incomplete, ignoree" % rest)
            if "--avi" in args:
                write_mjpg_avi(args[args.index("--avi") + 1], f, idx, w, h, fps or 10)
            if "--png" in args:
                outdir = os.path.splitext(path)[0] + "_jpg"
                os.makedirs(outdir, exist_ok=True)
                want = []
                for a in args[args.index("--png") + 1:]:
                    if a.startswith("--"):
                        break
                    want.append(int(a))
                for i in want:
                    if i >= n:
                        print("  image %d absente" % i)
                        continue
                    pos, ln = idx[i]
                    f.seek(pos)
                    q = os.path.join(outdir, "f%05d.jpg" % i)
                    open(q, "wb").write(f.read(ln))
                    print("  ->", q)
            if "--mp4" in args:
                import subprocess
                out = args[args.index("--mp4") + 1]
                cmd = ["ffmpeg", "-y", "-f", "image2pipe", "-vcodec", "mjpeg",
                       "-r", str(fps or 10), "-i", "-", "-pix_fmt", "yuv420p", out]
                print("  ", " ".join(cmd))
                pr = subprocess.Popen(cmd, stdin=subprocess.PIPE)
                for pos, ln in idx:
                    f.seek(pos)
                    pr.stdin.write(f.read(ln))
                pr.stdin.close()
                pr.wait()
            return

        n, rest, per = frame_count(path, w, h)
        print("%s : v%d, %dx%d, %d i/s" % (os.path.basename(path), ver, w, h, fps))
        print("  %d images completes (%.1f s), entete annonce %s"
              % (n, n / float(fps or 1), declared or "0 (enregistrement interrompu)"))
        if rest:
            print("  %d octets de queue : derniere image incomplete, ignoree" % rest)
        if declared and declared != n:
            print("  l'entete et la taille ne sont pas d'accord : c'est la taille qui fait foi")

        if "--avi" in args:
            write_avi(args[args.index("--avi") + 1], f, n, w, h, fps or 10, per, swap, flip)

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
                rgb = to_rgb888(f.read(per), swap)
                if flip:
                    rgb = b"".join(rgb[y * w * 3:(y + 1) * w * 3] for y in range(h - 1, -1, -1))
                write_png(p, w, h, rgb)
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
