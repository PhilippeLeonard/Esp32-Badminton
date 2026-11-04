#!/usr/bin/env python3
"""Serveur de match : le PC tient le score, la carte ne fait que detecter.

    python serveur.py [--port 8000]

L'adresse a donner a la carte est affichee au demarrage.
"""

import argparse
import json
import os
import pathlib
import re
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, quote, urlparse
from urllib.request import urlopen

import gray

ICI = pathlib.Path(__file__).parent
PAGE = ICI / "score_app.html"
SAUVEGARDE = ICI / "match.json"
CLIPS = ICI / "clips"

# Au-dela, les plus vieux sont oublies : la liste part dans chaque /etat.
MAX_ECHANGES = 5000
MAX_CLIPS = 500
# Un clip de 20 images QVGA en JPEG fait ~300 ko. La marge est large, mais
# elle borne ce qu'un POST peut faire tenir en memoire.
MAX_CLIP_OCTETS = 8 * 1024 * 1024
MAX_CONFIG_OCTETS = 64 * 1024

# Recuperation des fichiers restes sur la carte SD. Un .gray fait 768 ko/s
# d'enregistrement : la limite est la pour ne pas remplir le disque du PC
# par megarde, pas parce que le transfert y verrait une difficulte.
MAX_FICHIER_CARTE = 2 * 1024 * 1024 * 1024
BLOC_TRANSFERT = 64 * 1024
# Le lien passe par le wifi de l'ESP32 : lent, mais il doit avancer. Le
# delai porte sur chaque lecture, pas sur le transfert entier.
DELAI_CARTE = 30
NOM_CARTE = re.compile(r"^[A-Za-z0-9_.-]{1,48}$")

REGLAGES_DEFAUT = {
    "noms": ["Joueur A", "Joueur B"],
    # Axe du filet DANS L'IMAGE : "x" filet vertical, "y" filet horizontal.
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

ENTIERS = ("ligne", "cible", "ecart", "plafond", "jeux_pour_gagner",
           "largeur", "hauteur")

verrou = threading.Lock()
echanges = []
reglages = dict(REGLAGES_DEFAUT)
carte = {"ip": None, "vue": 0, "rssi": None, "ver": None,
         "ref": None, "zone": None}
clips = []
transferts = []              # [{id, fichier, etat, recu, total, sortie, erreur}]
transfert_verrou = threading.Lock()


# ------------------------------------------------------------------ clips

def ecrire_avi(chemin, frames, w, h, fps, jpeg):
    """Ecrit un AVI que VLC et Windows Media Player ouvrent directement.

    Deux cas, meme conteneur : des images JPEG sont recopiees telles quelles
    (MJPG), des images brutes deviennent un AVI 8 bits a palette de gris.
    Aucun codec a installer dans les deux cas.
    """
    n = len(frames)
    if not n or w <= 0 or h <= 0:
        return None
    if not jpeg and any(len(f) != w * h for f in frames):
        # Un DIB est lu ligne par ligne : une image plus courte ferait
        # ecrire des lignes vides sans que rien ne le signale.
        return None

    fps = fps or 1
    palette = b"" if jpeg else b"".join(bytes((i, i, i, 0)) for i in range(256))
    strf = 40 + len(palette)
    strl = 4 + 64 + 8 + strf
    hdrl = 4 + 64 + 8 + strl
    tete = 12 + 8 + hdrl + 12
    movi = sum(8 + len(f) + (len(f) & 1) for f in frames)
    gros = max(len(f) for f in frames)

    h_ = bytearray(tete)

    def put(o, b):
        h_[o:o + len(b)] = b

    def tg(k, v):
        return k + struct.pack("<I", v)

    put(0, tg(b"RIFF", tete - 8 + movi + 8 + 16 * n)); put(8, b"AVI ")
    put(12, tg(b"LIST", hdrl)); put(20, b"hdrl"); put(24, tg(b"avih", 56))
    put(32, struct.pack("<I", 1000000 // fps))
    put(36, struct.pack("<I", gros * fps))
    put(44, struct.pack("<I", 0x10)); put(48, struct.pack("<I", n))
    put(56, struct.pack("<I", 1)); put(60, struct.pack("<I", gros))
    put(64, struct.pack("<II", w, h))
    put(88, tg(b"LIST", strl)); put(96, b"strl"); put(100, tg(b"strh", 56))
    put(108, b"vids"); put(112, b"MJPG" if jpeg else b"DIB ")
    put(128, struct.pack("<I", 1)); put(132, struct.pack("<I", fps))
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

    Rend le pourcentage de pixels changes image par image, l'image ou ce
    pourcentage est maximal, et le centre des pixels changes a ce
    moment-la : c'est lui qui situe l'impact.
    """
    total = w * h
    if len(frames) < 2 or total <= 0:
        return None
    if any(len(f) != total for f in frames):
        return None

    ref = frames[0]
    pcts, centres = [], []
    for img in frames[1:]:
        n = cx = cy = 0
        for i in range(total):
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


# ----------------------------------------------- fichiers de la carte SD

def nom_carte_ok(nom):
    """Un nom de fichier de la racine de la carte, et rien d'autre.

    Il part dans une adresse vers la carte : un nom avec un / ou un .. y
    designerait autre chose que ce qu'on croit.
    """
    nom = (nom or "").lstrip("/")
    if not NOM_CARTE.match(nom) or ".." in nom:
        return None
    if not (nom.endswith(".gray") or nom.endswith(".avi")):
        return None
    return nom


def adresse_carte():
    """Hote de la carte, appris du battement de coeur ou d'une rafale."""
    with verrou:
        return carte["ip"]


def _transfert(tid):
    """A appeler avec transfert_verrou tenu."""
    for t in transferts:
        if t["id"] == tid:
            return t
    return None


def majt(tid, **champs):
    with transfert_verrou:
        t = _transfert(tid)
        if t:
            t.update(champs)


def recuperer_fichier(tid, hote, nom):
    """Telecharge un fichier de la carte, puis le convertit s'il le faut.

    Le telechargement va directement sur le disque, par blocs : un
    enregistrement peut peser des centaines de mega-octets, et le garder
    en memoire ferait tomber le serveur juste au moment ou on essaie
    enfin de recuperer les images.
    """
    cible = CLIPS / nom
    partiel = cible.with_name(cible.name + ".part")
    url = f"http://{hote}/download?f=/{quote(nom)}"
    try:
        CLIPS.mkdir(exist_ok=True)
        with urlopen(url, timeout=DELAI_CARTE) as r:
            total = r.headers.get("Content-Length")
            total = int(total) if total and total.isdigit() else 0
            if total > MAX_FICHIER_CARTE:
                raise ValueError(f"fichier de {total} octets, au-dela de la limite")
            majt(tid, total=total, etat="telechargement")

            recu = 0
            with open(partiel, "wb") as out:
                while True:
                    bloc = r.read(BLOC_TRANSFERT)
                    if not bloc:
                        break
                    recu += len(bloc)
                    if recu > MAX_FICHIER_CARTE:
                        raise ValueError("fichier plus gros que la limite")
                    out.write(bloc)
                    majt(tid, recu=recu)

        # La carte annonce une taille : si le compte n'y est pas, le
        # transfert a ete coupe et le fichier est incomplet.
        if total and recu != total:
            raise ValueError(f"transfert incomplet : {recu} octets sur {total}")
        partiel.replace(cible)
        print(f"[CARTE] {nom} recupere, {recu} o")

        sortie = cible
        if nom.endswith(".gray"):
            majt(tid, etat="conversion")
            avi = cible.with_suffix(".avi")
            info = gray.convertir_en_avi(
                cible, avi, lambda a, b: majt(tid, images=a, images_total=b))
            sortie = avi
            print(f"[CARTE] {nom} -> {avi.name} : {info['images']} images "
                  f"{info['largeur']}x{info['hauteur']}, {info['secondes']:.1f} s")
            majt(tid, info={k: info[k] for k in
                            ("largeur", "hauteur", "fps", "images", "secondes",
                             "images_declarees", "incomplete")})

        majt(tid, etat="termine", sortie=sortie.name, brut=cible.name,
             fin=time.time())
    except HTTPError as err:
        # 409 : la carte enregistre encore dans ce fichier.
        detail = ""
        try:
            detail = err.read().decode("utf-8", "replace").strip()
        except OSError:
            pass
        partiel.unlink(missing_ok=True)
        majt(tid, etat="echec", fin=time.time(),
             erreur=f"la carte repond {err.code}" + (f" : {detail}" if detail else ""))
    except (URLError, OSError, ValueError, gray.GrayInvalide) as err:
        partiel.unlink(missing_ok=True)
        majt(tid, etat="echec", fin=time.time(), erreur=str(err))
        print(f"[CARTE] {nom} : {err}")


def lancer_transfert(hote, nom):
    """Rend (tid, erreur). Le travail se fait dans un fil : un transfert de
    plusieurs minutes ne doit pas tenir une requete HTTP ouverte."""
    with transfert_verrou:
        for t in transferts:
            if t["fichier"] == nom and t["etat"] not in ("termine", "echec"):
                return None, "transfert deja en cours pour ce fichier"
        tid = f"t{int(time.time() * 1000) % 1000000000}{len(transferts)}"
        transferts.append({
            "id": tid, "fichier": nom, "etat": "demande", "recu": 0,
            "total": 0, "sortie": None, "brut": None, "erreur": None,
            "info": None, "images": 0, "images_total": 0,
            "debut": time.time(), "fin": None,
        })
        del transferts[:-40]
    threading.Thread(target=recuperer_fichier, args=(tid, hote, nom),
                     daemon=True).start()
    return tid, None


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
    """A appeler avec le verrou tenu.

    Ecriture dans un fichier temporaire puis remplacement : une coupure en
    pleine ecriture laisserait sinon un match.json tronque, donc illisible
    au redemarrage suivant.
    """
    tmp = SAUVEGARDE.with_suffix(".json.tmp")
    try:
        tmp.write_text(
            json.dumps({"echanges": echanges, "reglages": reglages}, indent=1),
            encoding="utf-8",
        )
        os.replace(tmp, SAUVEGARDE)
    except OSError as err:
        print("  sauvegarde impossible :", err)


def echange_valide(e):
    if not isinstance(e, dict) or e.get("j") not in (0, 1):
        return False
    return isinstance(e.get("t", 0), (int, float))


def appliquer_reglages(d):
    """A appeler avec le verrou tenu. Ignore en silence ce qui ne va pas :
    un reglage refuse vaut mieux qu'un serveur arrete."""
    for cle, val in d.items():
        if cle not in reglages:
            continue
        if cle == "noms" and isinstance(val, list) and len(val) == 2:
            reglages["noms"] = [str(v)[:20] for v in val]
        elif cle == "axe" and val in ("x", "y"):
            reglages["axe"] = val
        elif cle == "cote_a" and val in ("min", "max"):
            reglages["cote_a"] = val
        elif cle in ENTIERS:
            try:
                reglages[cle] = max(1, min(10000, int(val)))
            except (TypeError, ValueError):
                pass


def charger():
    if not SAUVEGARDE.exists():
        return
    try:
        d = json.loads(SAUVEGARDE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as err:
        print("  match.json illisible, on repart de zero :", err)
        return
    if not isinstance(d, dict):
        print("  match.json inattendu, on repart de zero")
        return

    # Un echange sans "j" valide ferait planter calculer() a chaque /etat,
    # donc a chaque chargement de la page.
    lus = d.get("echanges")
    gardes = [e for e in lus if echange_valide(e)] if isinstance(lus, list) else []
    echanges.extend(gardes[-MAX_ECHANGES:])

    r = d.get("reglages")
    if isinstance(r, dict):
        appliquer_reglages(r)

    perdus = (len(lus) if isinstance(lus, list) else 0) - len(gardes)
    print(f"  match repris : {len(echanges)} echanges"
          + (f" ({perdus} illisibles ignores)" if perdus else ""))


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
        try:
            self.send_response(code)
            self.send_header("Content-Type", type_mime)
            self.send_header("Content-Length", str(len(corps)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(corps)
        except (BrokenPipeError, ConnectionResetError):
            # Le telephone a ferme l'onglet pendant la reponse : ce n'est
            # pas une erreur du serveur, et la trace masquerait le reste.
            self.close_connection = True

    def _json(self, obj, code=200):
        self._envoyer(code, "application/json; charset=utf-8", json.dumps(obj))

    def _envoyer_fichier(self, chemin, nom_propose=None):
        """Sert un fichier par blocs.

        Un AVI converti depuis un .gray pese ce que pesait le .gray :
        le lire en entier en memoire pour le renvoyer, c'etait tenir
        plusieurs centaines de mega-octets par telechargement.
        """
        try:
            taille = chemin.stat().st_size
            with open(chemin, "rb") as src:
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(taille))
                self.send_header("Content-Disposition",
                                 f'attachment; filename="{nom_propose or chemin.name}"')
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                envoye = 0
                while envoye < taille:
                    bloc = src.read(BLOC_TRANSFERT)
                    if not bloc:
                        break
                    self.wfile.write(bloc)
                    envoye += len(bloc)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True
        except OSError as err:
            self._texte(500, f"lecture impossible : {err}")

    def _texte(self, code, msg):
        self._envoyer(code, "text/plain; charset=utf-8", msg)

    def _lire(self, maxi):
        """Corps de la requete, ou None s'il est absent ou trop gros."""
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            return None
        if n <= 0 or n > maxi:
            return None
        donnees = self.rfile.read(n)
        return donnees if len(donnees) == n else None

    def _corps(self):
        brut = self._lire(MAX_CONFIG_OCTETS)
        if brut is None:
            return {}
        try:
            d = json.loads(brut.decode("utf-8", "replace"))
        except ValueError:
            return {}
        return d if isinstance(d, dict) else {}

    def _params(self):
        # parse_qs decode %XX : une adresse de serveur ou un nom de fichier
        # arrive encode, et le comparer brut ne marche pas.
        q = urlparse(self.path).query
        return {k: v[0] for k, v in parse_qs(q, keep_blank_values=True).items()}

    def _vu(self):
        """A appeler avec le verrou tenu."""
        carte["ip"] = self.client_address[0]
        carte["vue"] = time.time()

    # -- routes

    def do_GET(self):
        chemin = urlparse(self.path).path
        params = self._params()
        if chemin == "/":
            if not PAGE.exists():
                self._texte(500, f"{PAGE.name} introuvable a cote de serveur.py")
                return
            try:
                self._envoyer(200, "text/html; charset=utf-8",
                              PAGE.read_text(encoding="utf-8"))
            except OSError as err:
                self._texte(500, f"{PAGE.name} illisible : {err}")
        elif chemin == "/etat":
            self._json(etat())
        elif chemin == "/vivante":
            self._vivante(params)
        elif chemin == "/clips":
            with verrou:
                self._json(clips[-50:])
        elif chemin == "/carte/fichiers":
            self._carte_fichiers()
        elif chemin == "/carte/transferts":
            with transfert_verrou:
                self._json(list(transferts))
        elif chemin.startswith("/clip/"):
            self._fichier_clip(chemin[len("/clip/"):])
        else:
            self._texte(404, "404")

    def do_POST(self):
        chemin = urlparse(self.path).path
        params = self._params()

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
        elif chemin == "/carte/recuperer":
            self._carte_recuperer(params)
        elif chemin == "/carte/supprimer":
            self._carte_supprimer(params)
        else:
            self._texte(404, "404")

    def do_OPTIONS(self):
        self._envoyer(204, "text/plain", "")

    # -- actions

    def _fichier_clip(self, nom):
        # Le nom est reduit a sa derniere composante : une adresse ne doit
        # pas pouvoir remonter l'arborescence.
        f = CLIPS / pathlib.Path(nom).name
        if not f.is_file():
            self._texte(404, "clip introuvable")
            return
        self._envoyer_fichier(f)

    def _carte_fichiers(self):
        """Liste ce que la carte garde sur sa SD, vue depuis le serveur.

        La page du serveur n'avait aucun moyen d'y acceder : il fallait
        ouvrir la page de la carte, et ce qu'on en rapportait etait un
        .gray qu'aucun lecteur n'ouvre.
        """
        hote = adresse_carte()
        if not hote:
            self._json({"ok": False, "raison": "adresse de la carte inconnue"}, 409)
            return
        try:
            with urlopen(f"http://{hote}/files", timeout=8) as r:
                liste = json.loads(r.read().decode("utf-8", "replace"))
        except (URLError, OSError, ValueError) as err:
            self._json({"ok": False, "raison": f"carte injoignable : {err}"}, 502)
            return
        if not isinstance(liste, list):
            self._json({"ok": False, "raison": "reponse inattendue de la carte"}, 502)
            return

        with transfert_verrou:
            etats = {t["fichier"]: t for t in transferts}
        for e in liste:
            if not isinstance(e, dict):
                continue
            nom = e.get("n") or ""
            e["convertible"] = nom.endswith(".gray")
            e["deja"] = (CLIPS / nom).is_file()
            t = etats.get(nom)
            e["transfert"] = t["etat"] if t else None
        self._json({"ok": True, "ip": hote, "fichiers": liste})

    def _carte_recuperer(self, params):
        nom = nom_carte_ok(params.get("f"))
        if not nom:
            self._json({"ok": False, "raison": "nom de fichier refuse"}, 400)
            return
        hote = adresse_carte()
        if not hote:
            self._json({"ok": False, "raison": "adresse de la carte inconnue"}, 409)
            return
        tid, err = lancer_transfert(hote, nom)
        if err:
            self._json({"ok": False, "raison": err}, 409)
            return
        print(f"[CARTE] recuperation de {nom} depuis {hote}")
        self._json({"ok": True, "id": tid, "fichier": nom})

    def _carte_supprimer(self, params):
        nom = nom_carte_ok(params.get("f"))
        if not nom:
            self._json({"ok": False, "raison": "nom de fichier refuse"}, 400)
            return
        hote = adresse_carte()
        if not hote:
            self._json({"ok": False, "raison": "adresse de la carte inconnue"}, 409)
            return
        try:
            with urlopen(f"http://{hote}/delete?f=/{quote(nom)}", timeout=8) as r:
                rep = r.read().decode("utf-8", "replace").strip()
        except (URLError, OSError) as err:
            self._json({"ok": False, "raison": f"carte injoignable : {err}"}, 502)
            return
        print(f"[CARTE] suppression de {nom} : {rep}")
        self._json({"ok": rep == "ok", "reponse": rep})

    def _vivante(self, params):
        """Battement de coeur de la carte, toutes les dix secondes. Sans
        lui, le serveur n'apprenait l'existence de la carte qu'a la
        premiere rafale : entre deux echanges la page affichait
        "carte jamais vue" alors que tout marchait."""
        def nombre(v):
            try:
                return int(v)
            except (TypeError, ValueError):
                return None

        with verrou:
            self._vu()
            carte["rssi"] = nombre(params.get("rssi"))
            carte["ver"] = params.get("ver") or None
            carte["ref"] = params.get("ref") == "1"
            carte["zone"] = params.get("zone") == "1"
        self._texte(200, "ok")

    def _clip(self, params):
        """Rafale envoyee par la carte : [u16 w][u16 h][u8 n] puis
        [u32 longueur][image] par image. Stockee telle quelle, et analysee
        si elle est brute - un clip JPEG demanderait un decodeur."""
        brut = self._lire(MAX_CLIP_OCTETS)
        if brut is None:
            self._texte(400, "corps vide ou trop gros")
            return
        if len(brut) < 5:
            self._texte(400, "trop court")
            return

        w = brut[0] | (brut[1] << 8)
        h = brut[2] | (brut[3] << 8)
        n = brut[4]
        if not (0 < w <= 4096 and 0 < h <= 4096):
            self._texte(400, "dimensions inattendues")
            return

        frames, o = [], 5
        while len(frames) < n and o + 4 <= len(brut):
            ln = int.from_bytes(brut[o:o + 4], "little")
            o += 4
            if ln == 0 or o + ln > len(brut):
                break
            frames.append(brut[o:o + ln])
            o += ln
        if not frames:
            self._texte(400, "aucune image lisible")
            return

        try:
            CLIPS.mkdir(exist_ok=True)
        except OSError as err:
            self._texte(500, f"dossier clips/ impossible : {err}")
            return

        # Deux rafales dans la meme seconde porteraient le meme nom et la
        # seconde ecraserait la premiere, clip et AVI compris.
        base = time.strftime("evt_%Y%m%d_%H%M%S")
        nom = base
        for k in range(1, 100):
            if not (CLIPS / (nom + ".clip")).exists():
                break
            nom = f"{base}_{k}"
        jpeg = frames[0][:2] == b"\xff\xd8"

        # On garde l'original - il peut servir a rejouer un algorithme -
        # mais ce qui s'ouvre d'un double-clic, c'est l'AVI ecrit a cote.
        chemin = CLIPS / (nom + ".clip")
        try:
            chemin.write_bytes(brut)
        except OSError as err:
            self._texte(500, f"ecriture impossible : {err}")
            return

        avi = None
        try:
            avi = ecrire_avi(CLIPS / (nom + ".avi"), frames, w, h, 10, jpeg)
        except (OSError, struct.error, ValueError) as err:
            print("[CLIP] conversion AVI impossible :", err)

        analyse = None
        if not jpeg:
            try:
                analyse = analyser_brut(frames, w, h)
            except (IndexError, ValueError) as err:
                print("[CLIP] analyse impossible :", err)

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
            self._vu()
            clips.append(info)
            del clips[:-MAX_CLIPS]
        print("[CLIP] %s : %d images %dx%d, %d o%s"
              % (chemin.name, len(frames), w, h, len(brut),
                 "" if analyse is None else
                 ", pic %.2f%% a l'image %d" % (analyse["pct_max"],
                                                analyse["image_max"])))
        self._json({"ok": True, "nom": chemin.name, "analyse": analyse})

    def _echange(self):
        d = self._corps()
        try:
            x, y = int(d.get("x", -1)), int(d.get("y", -1))
        except (TypeError, ValueError):
            x = y = -1
        rssi = d.get("rssi")

        with verrou:
            self._vu()
            carte["rssi"] = rssi if isinstance(rssi, (int, float)) else None
            appliquer_reglages({k: d[k] for k in ("largeur", "hauteur") if k in d})
            if x < 0 or y < 0:
                self._json({"ok": False, "raison": "position absente"}, 400)
                return
            j = gagnant(x, y, reglages)
            echanges.append({"t": time.time(), "j": j, "x": x, "y": y,
                             "src": "detection", "cellules": d.get("cellules")})
            del echanges[:-MAX_ECHANGES]
            sauver()
            e = calculer(echanges, reglages)
            nom = reglages["noms"][j]
        print(f"  echange x={x} y={y} -> point {nom}"
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
            del echanges[:-MAX_ECHANGES]
            sauver()
            nom = reglages["noms"][j]
        print(f"  point manuel -> {nom}")
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
            appliquer_reglages(d)
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
    if not 1 <= port <= 65535:
        raise SystemExit("port hors plage")

    charger()
    ip = adresse_lan()
    print("\n=== Compteur badminton - serveur du PC ===")
    print(f"  page   : http://{ip}:{port}/     (depuis le telephone aussi)")
    print(f"  carte  : http://ADRESSE_CARTE/server?url=http://{ip}:{port}")
    print("  Ctrl+C pour arreter\n")

    try:
        srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    except OSError as err:
        raise SystemExit(f"port {port} indisponible : {err}")
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n  arret")
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
