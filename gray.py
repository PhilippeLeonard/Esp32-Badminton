#!/usr/bin/env python3
"""Convertit les enregistrements /raw_NNNN.gray de la carte en AVI lisible.

Le .gray est du gris 8 bits sans aucun codec : aucun lecteur ne le connait.
L'AVI ecrit ici est un BI_RGB 8 bits a palette de gris, donc sans codec non
plus : VLC et Windows Media Player l'ouvrent directement, et il pese le
meme poids que le .gray.

    python gray.py raw_0001.gray                 # ce que contient le fichier
    python gray.py raw_0001.gray --avi video.avi # <-- lisible par VLC
    python gray.py raw_0001.gray --png 0 50 100  # quelques images

Le nombre d'images se deduit de la TAILLE du fichier, pas de l'entete :
celle-ci n'est corrigee qu'a l'arret propre de l'enregistrement, donc une
carte debranchee en pleine ecriture laisse un compteur a zero. Une
derniere image incomplete est ignoree.
"""

import argparse
import pathlib
import struct
import sys

ENTETE = 16
MAGIE = b"ESPG"
# Un .gray de QVGA a 10 i/s fait 768 ko/s. Les tampons restent petits : le
# fichier peut peser des centaines de mega-octets.
PAR_BLOC = 1 << 20


class GrayInvalide(ValueError):
    pass


def lire_entete(f):
    """Rend (largeur, hauteur, images_par_seconde, images_declarees)."""
    tete = f.read(ENTETE)
    if len(tete) < ENTETE or tete[:4] != MAGIE:
        raise GrayInvalide("ce n'est pas un enregistrement ESPG (magie absente)")
    w, h, fps, _ = struct.unpack_from("<HHHH", tete, 4)
    (declarees,) = struct.unpack_from("<I", tete, 12)
    if not (0 < w <= 4096 and 0 < h <= 4096):
        raise GrayInvalide(f"dimensions inattendues : {w}x{h}")
    return w, h, fps, declarees


def decrire(chemin):
    """Ce que le fichier contient vraiment, sans se fier a l'entete."""
    chemin = pathlib.Path(chemin)
    with open(chemin, "rb") as f:
        w, h, fps, declarees = lire_entete(f)
    taille = chemin.stat().st_size
    par_image = w * h
    images = max(0, (taille - ENTETE) // par_image)
    reste = max(0, taille - ENTETE) % par_image
    return {
        "largeur": w, "hauteur": h, "fps": fps or 1,
        "images": images,
        "images_declarees": declarees,
        "incomplete": reste,
        "octets": taille,
        "secondes": images / (fps or 1),
    }


# ----------------------------------------------------------------- AVI

def entete_avi(w, h, fps, n):
    """Entete complete d'un AVI BI_RGB 8 bits a palette de gris.

    Toutes les images font la meme taille, donc le nombre d'images et la
    taille du corps sont connus avant d'ecrire : l'entete part complete du
    premier coup, rien a corriger a la fin.
    """
    fps = fps or 1
    palette = b"".join(bytes((i, i, i, 0)) for i in range(256))
    strf = 40 + len(palette)
    strl = 4 + 64 + 8 + strf
    hdrl = 4 + 64 + 8 + strl
    tete = 12 + 8 + hdrl + 12
    par_image = w * h
    pad = par_image & 1
    movi = n * (8 + par_image + pad)

    b = bytearray(tete)

    def put(o, d):
        b[o:o + len(d)] = d

    def tg(k, v):
        return k + struct.pack("<I", v)

    put(0, tg(b"RIFF", tete - 8 + movi + 8 + 16 * n)); put(8, b"AVI ")
    put(12, tg(b"LIST", hdrl)); put(20, b"hdrl"); put(24, tg(b"avih", 56))
    put(32, struct.pack("<I", 1000000 // fps))
    put(36, struct.pack("<I", par_image * fps))
    put(44, struct.pack("<I", 0x10)); put(48, struct.pack("<I", n))
    put(56, struct.pack("<I", 1)); put(60, struct.pack("<I", par_image))
    put(64, struct.pack("<II", w, h))
    put(88, tg(b"LIST", strl)); put(96, b"strl"); put(100, tg(b"strh", 56))
    put(108, b"vids"); put(112, b"DIB ")
    put(128, struct.pack("<I", 1)); put(132, struct.pack("<I", fps))
    put(140, struct.pack("<I", n)); put(144, struct.pack("<I", par_image))
    put(148, struct.pack("<I", 0xFFFFFFFF))
    put(156, struct.pack("<HHHH", 0, 0, w, h))
    put(164, tg(b"strf", strf))
    put(172, struct.pack("<IiiHH", 40, w, h, 1, 8))
    put(188, struct.pack("<II", 0, par_image))       # BI_RGB
    put(204, struct.pack("<II", 256, 256)); put(212, palette)
    mo = 12 + 8 + hdrl
    put(mo, tg(b"LIST", 4 + movi)); put(mo + 8, b"movi")
    return bytes(b)


def convertir_en_avi(source, sortie, avance=None):
    """Ecrit l'AVI image par image, sans charger le fichier en memoire.

    `avance(faites, total)` est appele de temps en temps : un transfert de
    plusieurs centaines de mega-octets doit pouvoir se suivre.
    Rend le dictionnaire de decrire(), complete de "sortie".
    """
    source = pathlib.Path(source)
    sortie = pathlib.Path(sortie)
    info = decrire(source)
    w, h, fps, n = info["largeur"], info["hauteur"], info["fps"], info["images"]
    if n == 0:
        raise GrayInvalide("aucune image complete dans le fichier")

    par_image = w * h
    pad = bytes(par_image & 1)
    cle = b"00db"
    entree = cle + struct.pack("<I", par_image)

    # Ecriture dans un fichier temporaire : une conversion interrompue ne
    # doit pas laisser un AVI a moitie ecrit qui a l'air valide.
    tmp = sortie.with_name(sortie.name + ".part")
    idx = bytearray()
    off = 4
    try:
        with open(source, "rb") as src, open(tmp, "wb") as out:
            src.seek(ENTETE)
            out.write(entete_avi(w, h, fps, n))
            for k in range(n):
                image = src.read(par_image)
                if len(image) < par_image:
                    raise GrayInvalide("fichier plus court que sa taille annoncee")
                # Un DIB se lit du bas vers le haut.
                out.write(entree)
                for y in range(h - 1, -1, -1):
                    out.write(image[y * w:(y + 1) * w])
                if pad:
                    out.write(pad)
                idx += entree[:4] + struct.pack("<III", 0x10, off, par_image)
                off += 8 + par_image + len(pad)
                if avance and (k % 50 == 0 or k == n - 1):
                    avance(k + 1, n)
            out.write(b"idx1" + struct.pack("<I", 16 * n))
            out.write(bytes(idx))
        tmp.replace(sortie)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise

    info["sortie"] = str(sortie)
    return info


# ----------------------------------------------------------------- PNG

def ecrire_png(chemin, w, h, gris):
    """PNG en niveaux de gris, sans dependance."""
    import binascii
    import zlib

    lignes = bytearray()
    for y in range(h):
        lignes.append(0)                       # octet de filtre : aucun
        lignes += gris[y * w:(y + 1) * w]

    def bloc(genre, d):
        return (struct.pack(">I", len(d)) + genre + d
                + struct.pack(">I", binascii.crc32(genre + d) & 0xFFFFFFFF))

    pathlib.Path(chemin).write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + bloc(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
        + bloc(b"IDAT", zlib.compress(bytes(lignes), 6))
        + bloc(b"IEND", b""))


def extraire_png(source, indices, prefixe="image"):
    source = pathlib.Path(source)
    info = decrire(source)
    w, h, n = info["largeur"], info["hauteur"], info["images"]
    par_image = w * h
    ecrits = []
    with open(source, "rb") as f:
        for i in sorted(set(indices)):
            if not 0 <= i < n:
                print(f"  image {i} hors du fichier ({n} images)")
                continue
            f.seek(ENTETE + i * par_image)
            d = f.read(par_image)
            nom = source.with_name(f"{prefixe}_{i:05d}.png")
            ecrire_png(nom, w, h, d)
            ecrits.append(nom)
    return ecrits


# ----------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("fichier")
    ap.add_argument("--avi", metavar="SORTIE", nargs="?", const="",
                    help="ecrit un AVI lisible (defaut : a cote du .gray)")
    ap.add_argument("--png", metavar="N", type=int, nargs="+",
                    help="extrait ces images en PNG")
    a = ap.parse_args()

    src = pathlib.Path(a.fichier)
    if not src.is_file():
        raise SystemExit(f"{src} introuvable")
    try:
        info = decrire(src)
    except (GrayInvalide, OSError) as err:
        raise SystemExit(str(err))

    print(f"{src.name} : {info['largeur']}x{info['hauteur']}, "
          f"{info['fps']} i/s, {info['images']} images "
          f"({info['secondes']:.1f} s, {info['octets'] / 1048576:.1f} Mo)")
    if info["images_declarees"] != info["images"]:
        print(f"  entete : {info['images_declarees']} images annoncees - "
              "enregistrement interrompu, on se fie a la taille du fichier")
    if info["incomplete"]:
        print(f"  derniere image incomplete ({info['incomplete']} o) ignoree")

    if a.avi is not None:
        sortie = pathlib.Path(a.avi) if a.avi else src.with_suffix(".avi")

        def avance(faites, total):
            if faites == total:
                print(f"\r  conversion : {faites}/{total} images")
            else:
                print(f"\r  conversion : {faites}/{total} images", end="", flush=True)

        try:
            convertir_en_avi(src, sortie, avance)
        except (GrayInvalide, OSError) as err:
            raise SystemExit(f"conversion impossible : {err}")
        print(f"  -> {sortie}  ({sortie.stat().st_size / 1048576:.1f} Mo)")

    if a.png:
        for nom in extraire_png(src, a.png):
            print(f"  -> {nom}")

    if a.avi is None and not a.png:
        print("  (--avi pour un fichier lisible, --png pour des images)")


if __name__ == "__main__":
    main()
