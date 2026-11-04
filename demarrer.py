#!/usr/bin/env python3
"""Demarre tout : page, serveur, liaison avec la carte.

    python demarrer.py

Ou, sans ligne de commande : double-clic sur demarrer.bat (Windows),
demarrer.command (macOS) ou demarrer.sh (Linux).

Enchaine ce qu'il fallait faire a la main :
  1. regenere app_page.h si phone_app.html a change ;
  2. lance serveur.py sur un port libre ;
  3. cherche la carte (nom mDNS, derniere adresse connue, puis balayage
     du reseau local), lui dit ou poster ses rafales, et surveille cette
     liaison jusqu'a l'arret : la carte peut redemarrer ou changer
     d'adresse, elle est alors reliee a nouveau ;
  4. ouvre la page du serveur dans le navigateur.

Ctrl+C arrete le serveur.
"""

import argparse
import ipaddress
import json
import pathlib
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from urllib.error import URLError
from urllib.request import urlopen

ICI = pathlib.Path(__file__).resolve().parent
PAGE_SOURCE = ICI / "phone_app.html"
PAGE_GENEREE = ICI / "esp32_cam_recorder" / "app_page.h"
MEMOIRE = ICI / "carte.json"        # derniere adresse de carte qui a repondu

PORT_DEFAUT = 8000
NOMS_MDNS = ("esp32cam.local", "esp32cam")
# Adresse de l'AP de la carte : utile quand le PC est connecte dessus.
ADRESSE_AP = "192.168.4.1"


# ------------------------------------------------------------------ outils

def dire(etape, texte):
    print(f"  {etape:<9} {texte}", flush=True)


def adresse_lan():
    """Adresse par laquelle ce PC sort sur le reseau. Aucun paquet envoye :
    seul le choix de route du systeme est consulte."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def port_libre(depart):
    """Premier port libre a partir de depart. Un serveur deja lance, ou
    laisse par un essai precedent, ne doit pas empecher de demarrer."""
    for p in range(depart, depart + 20):
        s = socket.socket()
        # Meme option que ThreadingHTTPServer, sinon un port juste libere
        # (TIME_WAIT) passerait pour occupe et on changerait d'adresse
        # alors que serveur.py, lui, saurait le reprendre.
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", p))
            return p
        except OSError:
            continue
        finally:
            s.close()
    raise SystemExit(f"aucun port libre entre {depart} et {depart + 19}")


# ---------------------------------------------------------------- la page

def regenerer_page():
    """app_page.h est genere depuis phone_app.html. Oublier de le
    regenerer fait servir l'ancienne page par la carte, ce que la page
    signale ensuite en haut a droite."""
    if not PAGE_SOURCE.is_file():
        dire("page", f"{PAGE_SOURCE.name} absent, rien a regenerer")
        return
    if (PAGE_GENEREE.is_file()
            and PAGE_GENEREE.stat().st_mtime >= PAGE_SOURCE.stat().st_mtime):
        dire("page", "app_page.h a jour")
        return
    r = subprocess.run([sys.executable, str(ICI / "make_page.py")],
                       cwd=ICI, capture_output=True, text=True)
    if r.returncode == 0:
        dire("page", "app_page.h regenere - a reflasher pour en profiter")
    else:
        dire("page", "make_page.py a echoue : " + (r.stderr or r.stdout).strip())


# -------------------------------------------------------------- la carte

def interroger(hote, delai=1.5):
    """Rend le JSON de /motion si l'hote est bien une carte, sinon None."""
    try:
        with urlopen(f"http://{hote}/motion", timeout=delai) as f:
            d = json.loads(f.read().decode("utf-8", "replace"))
    except (URLError, OSError, ValueError, socket.timeout):
        return None
    return d if isinstance(d, dict) and "ver" in d else None


def port_ouvert(ip, delai=0.35):
    s = socket.socket()
    s.settimeout(delai)
    try:
        return s.connect_ex((ip, 80)) == 0
    except OSError:
        return False
    finally:
        s.close()


def candidats_memorises():
    try:
        d = json.loads(MEMOIRE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    ip = d.get("ip") if isinstance(d, dict) else None
    return [ip] if isinstance(ip, str) and ip else []


def memoriser(ip):
    try:
        MEMOIRE.write_text(json.dumps({"ip": ip}), encoding="utf-8")
    except OSError:
        pass


def balayer(ip_locale, stop):
    """Essaie les 254 adresses du /24 local. Le port 80 est teste d'abord
    parce qu'un refus de connexion revient tout de suite, alors qu'une
    adresse vide ne repond jamais : interroger /motion partout prendrait
    plusieurs minutes."""
    try:
        reseau = ipaddress.ip_network(ip_locale + "/24", strict=False)
    except ValueError:
        return None
    hotes = [str(h) for h in reseau.hosts() if str(h) != ip_locale]
    with ThreadPoolExecutor(max_workers=64) as pool:
        for ip, ouvert in zip(hotes, pool.map(port_ouvert, hotes)):
            if stop.is_set():
                return None
            if not ouvert:
                continue
            if interroger(ip, 1.0):
                return ip
    return None


def trouver_carte(ip_locale, impose, stop):
    essais = []
    if impose:
        essais = [impose]
    else:
        essais = list(NOMS_MDNS) + candidats_memorises() + [ADRESSE_AP]

    vus = set()
    for hote in essais:
        if stop.is_set():
            return None
        if hote in vus:
            continue
        vus.add(hote)
        if interroger(hote):
            return hote
    if impose:
        return None

    dire("carte", "balayage du reseau local...")
    return balayer(ip_locale, stop)


def relier(hote, mon_url):
    """Dit a la carte ou poster ses rafales. Sans cette etape la carte
    ecrit sur sa carte SD et rien n'arrive au serveur."""
    from urllib.parse import quote
    try:
        with urlopen(f"http://{hote}/server?url={quote(mon_url, safe='')}",
                     timeout=4) as f:
            return f.read().decode("utf-8", "replace").strip()
    except (URLError, OSError, socket.timeout) as err:
        return f"ECHEC: {err}"


def url_courante(hote):
    """Ce que la carte a comme adresse de serveur. None si elle ne repond
    plus du tout - c'est ainsi qu'on voit un redemarrage ou une perte de
    wifi, et qu'on peut relier a nouveau."""
    try:
        with urlopen(f"http://{hote}/server", timeout=3) as f:
            return f.read().decode("utf-8", "replace").strip()
    except (URLError, OSError, socket.timeout):
        return None


def decrire_carte(hote, etat):
    dire("carte", f"trouvee sur http://{hote}  (firmware {etat.get('ver', '?')})")
    if not etat.get("cam", True):
        dire("carte", "ATTENTION : la camera n'a pas demarre sur la carte")
    if not etat.get("sd", True):
        dire("carte", "pas de carte SD : sans serveur joignable, les rafales "
                      "seraient perdues")
    manque = []
    if not etat.get("ref"):
        manque.append("la reference")
    if not etat.get("zone"):
        manque.append("le terrain")
    if manque:
        dire("a faire", "sur la page : prendre " + " et ".join(manque)
             + " (sans les deux, la carte n'envoie rien)")


def preparer_carte(ip_locale, port, impose, stop):
    """Tourne a cote du serveur, sans fin.

    Un seul passage ne suffisait pas : la carte redemarre (coupure USB,
    brownout a l'init camera), le hotspot redistribue les baux, et plus
    rien n'arrivait au serveur alors que les deux tournaient. On verifie
    donc en continu que la carte repond et qu'elle pointe bien ici, et on
    la relie des que ce n'est plus le cas."""
    hote = None
    attente = 5.0
    perdue_dite = False

    while not stop.is_set():
        if hote is None:
            hote = trouver_carte(ip_locale, impose, stop)
            if hote is None:
                if not perdue_dite:
                    dire("carte", "introuvable - le serveur tourne, la page "
                                  "permet de saisir l'adresse a la main ; "
                                  "recherche en arriere-plan")
                    perdue_dite = True
                stop.wait(attente)
                attente = min(attente * 2, 60.0)
                continue
            attente = 5.0
            perdue_dite = False
            decrire_carte(hote, interroger(hote) or {})
            memoriser(hote)

        # L'adresse du PC peut changer en cours de route : c'est le hotspot
        # du telephone qui distribue les baux.
        mon_url = f"http://{adresse_lan()}:{port}"
        actuelle = url_courante(hote)
        if actuelle is None:
            dire("carte", "ne repond plus - nouvelle recherche")
            hote = None
            stop.wait(2)
            continue
        if actuelle.rstrip("/") != mon_url:
            reponse = relier(hote, mon_url)
            if reponse.startswith("ECHEC"):
                dire("liaison", reponse)
                hote = None
                stop.wait(2)
                continue
            dire("liaison", f"la carte postera ses rafales a {reponse or mon_url}")
        stop.wait(15)


# ------------------------------------------------------------------ main

def attendre_serveur(port, delai=10.0):
    fin = time.time() + delai
    while time.time() < fin:
        try:
            with urlopen(f"http://127.0.0.1:{port}/etat", timeout=1) as f:
                f.read()
            return True
        except (URLError, OSError, socket.timeout):
            time.sleep(0.25)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=PORT_DEFAUT)
    ap.add_argument("--carte", metavar="ADRESSE",
                    help="adresse de la carte, au lieu de la chercher")
    ap.add_argument("--sans-navigateur", action="store_true")
    ap.add_argument("--sans-carte", action="store_true",
                    help="ne pas chercher la carte du tout")
    a = ap.parse_args()

    if sys.version_info < (3, 8):
        raise SystemExit("Python 3.8 ou plus recent est necessaire")
    for f in ("serveur.py", "score_app.html"):
        if not (ICI / f).is_file():
            raise SystemExit(f"{f} introuvable a cote de demarrer.py")

    print("\n=== Badminton - demarrage ===")
    regenerer_page()

    port = port_libre(a.port)
    if port != a.port:
        dire("port", f"{a.port} occupe, on prend {port}")
    ip_locale = adresse_lan()
    url = f"http://{ip_locale}:{port}/"

    srv = subprocess.Popen([sys.executable, str(ICI / "serveur.py"),
                            "--port", str(port)], cwd=ICI)
    stop = threading.Event()
    chercheur = None
    try:
        if not attendre_serveur(port):
            dire("serveur", "pas de reponse - voir les lignes ci-dessus")
        else:
            dire("serveur", url + "     (depuis le telephone aussi)")

        if not a.sans_carte:
            chercheur = threading.Thread(
                target=preparer_carte,
                args=(ip_locale, port, a.carte, stop), daemon=True)
            chercheur.start()

        if not a.sans_navigateur:
            # Sur une machine sans navigateur, open() rend simplement False.
            try:
                webbrowser.open(url)
            except webbrowser.Error:
                pass

        srv.wait()
    except KeyboardInterrupt:
        print("\n  arret demande", flush=True)
    finally:
        stop.set()
        if srv.poll() is None:
            srv.terminate()
            try:
                srv.wait(timeout=5)
            except subprocess.TimeoutExpired:
                srv.kill()
        if chercheur:
            chercheur.join(timeout=2)
        print("  arrete")


if __name__ == "__main__":
    main()
