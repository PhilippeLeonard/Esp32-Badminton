#!/usr/bin/env python3
"""Serveur de match : le portable tient le score, la carte ne fait que detecter.

La carte POST un petit JSON sur /echange quand un echange se termine. Tout le
reste - regles du badminton, score, serveur, historique, annulation, page -
est calcule ici. La carte n'a plus rien a tenir a jour, et rien a reconstruire
quand on recharge la page.

    python serveur.py [--port 8000]

L'adresse a recopier dans config.h (SERVEUR_URL) est affichee au demarrage.
"""

import argparse
import json
import pathlib
import struct
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ICI = pathlib.Path(__file__).parent
PAGE = ICI / "score_app.html"
SAUVEGARDE = ICI / "match.json"
CLIPS = ICI / "clips"

REGLAGES_DEFAUT = {
    "noms": ["Joueur A", "Joueur B"],
    # Axe du filet DANS L'IMAGE : "x" = filet vertical, "y" = filet horizontal.
    "axe": "x",
    "ligne": 160,            # pixel du filet sur cet axe
    "cote_a": "min",         # A occupe le cote ou la coordonnee est < ligne
    "largeur": 320,
    "hauteur": 240,
    "cible": 21,
    "ecart": 2,
    "plafond": 30,           # a 29-29, le point suivant gagne le jeu
    "jeux_pour_gagner": 2,
}

verrou = threading.Lock()
echanges = []                # [{t, j, x, y, src, cellules}]
reglages = dict(REGLAGES_DEFAUT)
carte = {"ip": None, "vue": 0, "rssi": None}
clips = []                   # [{nom, t, n, w, h, in, band, cx, cy, analyse}]


# ------------------------------------------------------------------ clips

def ecrire_avi(chemin, frames, w, h, fps, jpeg):
    """Ecrit un AVI que VLC et Windows Media Player ouvrent directement.

    Le corps recu de la carte est un format maison : personne ne sait le
    lire, et demander un outil a chaque clip n'a pas de sens. On convertit
    donc a l'arrivee, une fois pour toutes.

    Deux cas, meme conteneur : des images JPEG sont recopiees telles
    quelles (MJPG), des images brutes deviennent un AVI 8 bits avec une
    palette de gris. Aucun codec a installer dans les deux cas."""
    n = len(frames)
    if not n:
        return None
    palette = b"" if jpeg else b"".join(bytes((i, i, i, 0)) for i in range(256))
    strf = 40 + len(palette)
    strl = 4 + 64 + 8 + strf
    hdrl = 4 + 64 + 8 + strl
    tete = 12 + 8 + hdrl + 12
    movi = sum(8 + len(f) + (len(f) & 1) for f in frames)
    gros = max(len(f) for f in frames)

    h_ = bytearray(tete)
    def put(o, b): h_[o:o + len(b)] = b
    def tg(k, v): return k + struct.pack("<I", v)
    put(0, tg(b"RIFF", tete - 8 + movi + 8 + 16 * n)); put(8, b"AVI ")
    put(12, tg(b"LIST", hdrl)); put(20, b"hdrl"); put(24, tg(b"avih", 56))
    put(32, struct.pack("<I", 1000000 // (fps or 1)))
    put(36, struct.pack("<I", gros * (fps or 1)))
    put(44, struct.pack("<I", 0x10)); put(48, struct.pack("<I", n))
    put(56, struct.pack("<I", 1)); put(60, struct.pack("<I", gros))
    put(64, struct.pack("<II", w, h))
    put(88, tg(b"LIST", strl)); put(96, b"strl"); put(100, tg(b"strh", 56))
    put(108, b"vids"); put(112, b"MJPG" if jpeg else b"DIB ")
    put(128, struct.pack("<I", 1)); put(132, struct.pack("<I", fps or 1))
    put(140, struct.pack("<I", n)); put(144, struct.pack("<I", gros))
    put(148, struct.pack("<I", 0xFFFFFFFF))
    put(156, struct.pack("<HHHH", 0, 0, w, h))
    put(164, tg(b"strf", strf))
    put(172, struct.pack("<IiiHH", 40, w, h, 1, 24 if jpeg else 8))
    if jpeg:
        put(188, b"MJPG"); put(192, struct.pack("<I", w * h * 3))
    else:
        put(188, struct.pack("<II", 0, w * h))
        put(204, struct.pack("<II", 256, 256)); put(212, palette)
    mo = 12 + 8 + hdrl
    put(mo, tg(b"LIST", 4 + movi)); put(mo + 8, b"movi")

    cle = b"00dc" if jpeg else b"00db"
    idx = bytearray()
    off = 4
    with open(chemin, "wb") as o:
        o.write(bytes(h_))
        for f in frames:
            d = f
            if not jpeg:
                # un DIB se lit du bas vers le haut
                d = b"".join(f[y * w:(y + 1) * w] for y in range(h - 1, -1, -1))
            o.write(cle + struct.pack("<I", len(d)))
            o.write(d)
            if len(d) & 1:
                o.write(bytes(1))
            idx += cle + struct.pack("<III", 0x10, off, len(d))
            off += 8 + len(d) + (len(d) & 1)
        o.write(b"idx1" + struct.pack("<I", 16 * n))
        o.write(bytes(idx))
    return chemin

def analyser_brut(frames, w, h, seuil=30):
    """Compare chaque image a la premiere, qui sert de reference.

    Travaille sur des images brutes en niveaux de gris : pas de decodage,
    donc pas de dependance, et les valeurs sont celles du capteur. Rend le
    pourcentage de pixels changes image par image, l'image ou ce
    pourcentage est maximal, et le centre des pixels changes a ce
    moment-la : c'est lui qui situe l'impact."""
    if len(frames) < 2:
        return None
    ref = frames[0]
    total = w * h
    pcts, centres = [], []
    for img in frames[1:]:
        n = cx = cy = 0
        for i in range(0, total, 1):
            d = img[i] - ref[i]
            if (d if d >= 0 else -d) > seuil:
                n += 1
                cx += i % w
                cy += i // w
        pcts.append(100.0 * n / total)
        centres.append((cx // n, cy // n) if n else (None, None))
    k = max(range(len(pcts)), key=lambda i: pcts[i])
    return {
        "pics": [round(v, 2) for v in pcts],
        "image_max": k + 1,
        "pct_max": round(pcts[k], 2),
        "centre": centres[k],
    }


# ---------------------------------------------------------------- regles

def gagnant(x, y, r):
    """Le volant tombe dans un camp : le point est pour l'autre."""
    c = x if r["axe"] == "x" else y
    cote_min = c < r["ligne"]
    perdant = 0 if cote_min == (r["cote_a"] == "min") else 1
    return 1 - perdant


def calculer(liste, r):
    """Tout le score est recalcule depuis la liste des echanges.

    C'est ce qui rend l'annulation exacte : on retire le dernier echange et
    on refait le compte, au lieu de defaire des additions a l'envers.
    """
    score, jeux, serveur, fini = [0, 0], [0, 0], 0, False
    for e in liste:
        if fini:
            break
        j = e["j"]
        score[j] += 1
        serveur = j
        haut, bas = max(score), min(score)
        if (haut >= r["cible"] and haut - bas >= r["ecart"]) or haut >= r["plafond"]:
            jeux[j] += 1
            score = [0, 0]
            if jeux[j] >= r["jeux_pour_gagner"]:
                fini = True
    return {
        "score": score,
        "jeux": jeux,
        "serveur": serveur,
        "fini": fini,
        "vainqueur": (jeux.index(max(jeux)) if fini else None),
        "echanges": len(liste),
    }


def etat():
    with verrou:
        e = calculer(echanges, reglages)
        e["reglages"] = dict(reglages)
        e["journal"] = echanges[-14:][::-1]
        e["carte"] = dict(carte)
        e["carte"]["depuis"] = (time.time() - carte["vue"]) if carte["vue"] else None
        return e


def sauver():
    try:
        SAUVEGARDE.write_text(
            json.dumps({"echanges": echanges, "reglages": reglages}, indent=1),
            encoding="utf-8",
        )
    except OSError as err:
        print("  sauvegarde impossible :", err)


def charger():
    if not SAUVEGARDE.exists():
        return
    try:
        d = json.loads(SAUVEGARDE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as err:
        print("  match.json illisible, on repart de zero :", err)
        return
    echanges.extend(d.get("echanges", []))
    reglages.update({k: v for k, v in d.get("reglages", {}).items() if k in reglages})
    print(f"  match repris : {len(echanges)} echanges")


# ---------------------------------------------------------------- serveur

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "badminton/1.0"

    def log_message(self, *a):
        pass                              # on imprime nos propres lignes

    # -- sorties

    def _envoyer(self, code, type_mime, corps):
        if isinstance(corps, str):
            corps = corps.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", type_mime)
        self.send_header("Content-Length", str(len(corps)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(corps)

    def _json(self, obj, code=200):
        self._envoyer(code, "application/json; charset=utf-8", json.dumps(obj))

    def _corps(self):
        n = int(self.headers.get("Content-Length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8", "replace"))
        except ValueError:
            return {}

    # -- routes

    def do_GET(self):
        chemin = self.path.split("?")[0]
        if chemin == "/":
            if not PAGE.exists():
                self._envoyer(500, "text/plain; charset=utf-8",
                              f"{PAGE.name} introuvable a cote de serveur.py")
                return
            self._envoyer(200, "text/html; charset=utf-8",
                          PAGE.read_text(encoding="utf-8"))
        elif chemin == "/etat":
            self._json(etat())
        elif chemin == "/clips":
            with verrou:
                self._json(clips[-50:])
        elif chemin.startswith("/clip/"):
            self._fichier_clip(chemin[len("/clip/"):])
        else:
            self._envoyer(404, "text/plain; charset=utf-8", "404")

    def do_POST(self):
        chemin = self.path.split("?")[0]
        params = {}
        if "?" in self.path:
            for part in self.path.split("?", 1)[1].split("&"):
                cle, _, val = part.partition("=")
                params[cle] = val

        if chemin == "/clip":
            self._clip(params)
        elif chemin == "/echange":
            self._echange()
        elif chemin == "/point":
            self._point(params)
        elif chemin == "/annuler":
            self._annuler()
        elif chemin == "/reset":
            self._reset()
        elif chemin == "/config":
            self._config()
        else:
            self._envoyer(404, "text/plain; charset=utf-8", "404")

    def do_OPTIONS(self):
        self._envoyer(204, "text/plain", "")

    # -- actions

    def _fichier_clip(self, nom):
        """Telechargement d'un clip recu. Le nom est reduit a sa derniere
        composante : une adresse ne doit pas pouvoir remonter l'arborescence."""
        f = CLIPS / pathlib.Path(nom).name
        if not f.is_file():
            self._envoyer(404, "text/plain; charset=utf-8", "clip introuvable")
            return
        self._envoyer(200, "application/octet-stream", f.read_bytes())

    def _clip(self, params):
        """Rafale envoyee par la carte : [u16 w][u16 h][u8 n] puis
        [u32 longueur][image] par image. Stockee telle quelle, et analysee
        si elle est brute - un clip JPEG demanderait un decodeur."""
        n_oct = int(self.headers.get("Content-Length") or 0)
        if not n_oct:
            self._envoyer(400, "text/plain", "corps vide")
            return
        brut = self.rfile.read(n_oct)
        if len(brut) < 5:
            self._envoyer(400, "text/plain", "trop court")
            return

        w = brut[0] | (brut[1] << 8)
        h = brut[2] | (brut[3] << 8)
        n = brut[4]
        frames, o = [], 5
        while len(frames) < n and o + 4 <= len(brut):
            ln = int.from_bytes(brut[o:o + 4], "little")
            o += 4
            if o + ln > len(brut):
                break
            frames.append(brut[o:o + ln])
            o += ln

        CLIPS.mkdir(exist_ok=True)
        nom = time.strftime("evt_%Y%m%d_%H%M%S")
        jpeg = bool(frames) and frames[0][:2] == bytes((0xFF, 0xD8))

        """Le corps recu est un format maison. On garde l'original - il
        peut servir a rejouer un algorithme - mais ce qui s'ouvre d'un
        double-clic, c'est l'AVI ecrit a cote."""
        chemin = CLIPS / (nom + ".clip")   # format maison, pas un .mjpg
        chemin.write_bytes(brut)
        avi = None
        try:
            fps = 10
            avi = ecrire_avi(CLIPS / (nom + ".avi"), frames, w, h, fps, jpeg)
        except Exception as e:
            print("[CLIP] conversion AVI impossible :", e)

        analyse = None
        if not jpeg and frames and len(frames[0]) == w * h:
            analyse = analyser_brut(frames, w, h)

        info = {
            "nom": chemin.name,
            "avi": (avi.name if avi else None),
            "t": time.time(), "n": len(frames),
            "w": w, "h": h,
            "in": params.get("in"), "band": params.get("band"),
            "cx": params.get("cx"), "cy": params.get("cy"),
            "ms": params.get("ms"),
            "analyse": analyse,
        }
        with verrou:
            carte["ip"] = self.client_address[0]
            carte["vue"] = time.time()
            clips.append(info)
        print("[CLIP] %s : %d images %dx%d, %d o%s"
              % (chemin.name, len(frames), w, h, len(brut),
                 "" if analyse is None else
                 ", pic %.2f%% a l'image %d" % (analyse["pct_max"], analyse["image_max"])))
        self._json({"ok": True, "nom": chemin.name, "analyse": analyse})

    def _echange(self):
        d = self._corps()
        x, y = int(d.get("x", -1)), int(d.get("y", -1))
        with verrou:
            carte["ip"] = self.client_address[0]
            carte["vue"] = time.time()
            carte["rssi"] = d.get("rssi")
            for cle in ("largeur", "hauteur"):
                if d.get(cle):
                    reglages[cle] = int(d[cle])
            if x < 0 or y < 0:
                self._json({"ok": False, "raison": "position absente"}, 400)
                return
            j = gagnant(x, y, reglages)
            echanges.append({"t": time.time(), "j": j, "x": x, "y": y,
                             "src": "detection", "cellules": d.get("cellules")})
            sauver()
            e = calculer(echanges, reglages)
        print(f"  echange x={x} y={y} -> point {reglages['noms'][j]}"
              f"   {e['score'][0]}-{e['score'][1]}")
        self._json({"ok": True, "point": j, "score": e["score"]})

    def _point(self, params):
        try:
            j = int(params.get("j", ""))
        except ValueError:
            j = -1
        if j not in (0, 1):
            self._json({"ok": False, "raison": "joueur inconnu"}, 400)
            return
        with verrou:
            echanges.append({"t": time.time(), "j": j, "x": None, "y": None,
                             "src": "manuel", "cellules": None})
            sauver()
        print(f"  point manuel -> {reglages['noms'][j]}")
        self._json(etat())

    def _annuler(self):
        with verrou:
            retire = echanges.pop() if echanges else None
            if retire:
                sauver()
        print("  annulation" if retire else "  rien a annuler")
        self._json(etat())

    def _reset(self):
        with verrou:
            echanges.clear()
            sauver()
        print("  nouveau match")
        self._json(etat())

    def _config(self):
        d = self._corps()
        with verrou:
            for cle, val in d.items():
                if cle not in reglages:
                    continue
                if cle == "noms" and isinstance(val, list) and len(val) == 2:
                    reglages["noms"] = [str(v)[:20] for v in val]
                elif cle == "axe" and val in ("x", "y"):
                    reglages["axe"] = val
                elif cle == "cote_a" and val in ("min", "max"):
                    reglages["cote_a"] = val
                elif cle in ("ligne", "cible", "ecart", "plafond",
                             "jeux_pour_gagner", "largeur", "hauteur"):
                    try:
                        reglages[cle] = max(1, int(val))
                    except (TypeError, ValueError):
                        pass
            sauver()
        self._json(etat())


def adresse_lan():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))        # aucun paquet envoye, juste le routage
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    port = ap.parse_args().port

    charger()
    ip = adresse_lan()
    print("\n=== Compteur badminton - serveur du portable ===")
    print(f"  page      : http://{ip}:{port}/     (depuis le telephone aussi)")
    print(f"  config.h  : #define SERVEUR_URL \"http://{ip}:{port}/echange\"")
    print("  Ctrl+C pour arreter\n")

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  arret")
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
