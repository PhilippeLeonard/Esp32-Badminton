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
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ICI = pathlib.Path(__file__).parent
PAGE = ICI / "score_app.html"
SAUVEGARDE = ICI / "match.json"

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
        else:
            self._envoyer(404, "text/plain; charset=utf-8", "404")

    def do_POST(self):
        chemin = self.path.split("?")[0]
        params = {}
        if "?" in self.path:
            for part in self.path.split("?", 1)[1].split("&"):
                cle, _, val = part.partition("=")
                params[cle] = val

        if chemin == "/echange":
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
