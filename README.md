# Esp32-Badminton

Détecteur de changement sur Freenove ESP32-WROVER CAM. La carte compare
chaque image à une référence, n'envoie qu'un **pourcentage**, et ne
transmet des images que lorsqu'un événement se termine.

Le travail est réparti : le PC trace le terrain et analyse les rafales,
la carte se contente de mesurer et d'envoyer. **Sans serveur, la carte
ne détecte rien** et se limite à l'enregistrement brut — c'est le mode
dégradé voulu en prototype, pas une panne.

## Disposition du dépôt

```
demarrer.py                  tout démarrer d'un coup  <-- commencer ici
demarrer.bat                 double-clic (Windows)
demarrer.command             double-clic (macOS)
demarrer.sh                  double-clic (Linux)
esp32_cam_recorder/
    esp32_cam_recorder.ino   le firmware  <-- ouvrir CE dossier dans l'IDE
    app_page.h               généré, ne pas éditer à la main
phone_app.html               la page servie par la carte (source à modifier)
make_page.py                 phone_app.html  ->  esp32_cam_recorder/app_page.h
serveur.py                   le serveur : score, terrain, réception des rafales
score_app.html               la page du serveur
esp32_cam_simple/
    esp32_cam_simple.ino     croquis à part, indépendant du détecteur
tools/replay.py              rejoue la détection sur un enregistrement .gray
tools/rgb_tool.py            convertit les .rgb et .mjpg en AVI lisible
```

L'IDE Arduino n'ouvre un croquis que s'il est dans un dossier portant son
nom. C'est pourquoi le `.ino` est dans `esp32_cam_recorder/` et non à la
racine : ouvert depuis la racine, l'IDE en fabrique une copie ailleurs, et
on finit par éditer un fichier pendant qu'on en flashe un autre.

## Mise en route

Alimenter la carte, puis **un seul geste** sur le PC :

| système | quoi faire |
| --- | --- |
| Windows | double-clic sur `demarrer.bat` |
| macOS | double-clic sur `demarrer.command` |
| Linux | double-clic sur `demarrer.sh`, ou `python3 demarrer.py` |

`demarrer.py` enchaîne tout ce qu'il fallait faire à la main :

1. régénère `app_page.h` si `phone_app.html` a changé ;
2. lance `serveur.py` sur un port libre ;
3. **trouve la carte** — nom mDNS `esp32cam.local`, puis la dernière
   adresse connue, puis `192.168.4.1`, puis un balayage du réseau local —
   et lui dit où poster ses rafales ;
4. ouvre la page du serveur dans le navigateur.

Ctrl+C arrête le serveur. Options utiles :

```bash
python demarrer.py --carte 192.168.1.57   # adresse connue, pas de recherche
```

```bash
python demarrer.py --port 9000 --sans-navigateur
```

Il reste **deux choses à faire sur la page**, celles qui demandent de
regarder l'image, dans le volet *Terrain et référence* :

1. **Prendre la référence sur la carte** — scène au repos, l'exposition
   se verrouille et huit images sont moyennées.
2. Cliquer les **quatre coins** du terrain sur l'image affichée, régler
   la marge, puis **Envoyer le terrain à la carte**.

Tant que la référence **et** le terrain ne sont pas définis, la carte
n'envoie rien : la page le rappelle sous les boutons.

### À la main, sans `demarrer.py`

```bash
python make_page.py              # si phone_app.html a changé
python serveur.py --port 8000
```

puis, dans la page, saisir l'adresse de la carte et cliquer
**1 · Relier la carte à ce serveur** (ce que fait sinon `demarrer.py`).

Le terrain se trace sur le PC : la carte ne reçoit que huit nombres.

### Quand un événement se ferme

Les vingt dernières images partent en un seul `POST /clip`, avec les
mesures dans l'adresse — pourcentages dans le terrain et dans la marge,
centre des pixels changés, durée. Ces mesures sont celles du **pic** de
l'événement, pas celles de la scène calme qui vient de le fermer.

**Si le POST échoue, la rafale est écrite sur la carte SD** : un
événement ne se perd pas parce que le PC est éteint. La carte ne tente
même pas le POST quand le wifi est tombé, pour ne pas bloquer la
détection pendant le délai de connexion.

Le serveur range le clip dans `clips/` et écrit à côté un `.avi` qu'un
lecteur ouvre directement. S'il est brut, il en sort aussi le pic de
changement et son centre. Un clip JPEG est stocké sans être analysé : le
décoder demanderait Pillow.

## Modifier l'interface

```bash
python make_page.py      # régénère app_page.h depuis phone_app.html
```

À lancer **avant** de recompiler, sinon la carte sert l'ancienne page. En
cas d'oubli, la page affiche `page vX / carte vY` en orange en haut à
droite : les deux versions ne correspondent pas.

## Réglages IDE

- Carte : **ESP32 Wrover Module**
- PSRAM : **Enabled** (obligatoire, les tampons vivent en PSRAM)
- Partition Scheme : **Huge APP (3MB No OTA)**

## Les deux pages

Il y a **deux** pages, et elles ne servent pas à la même chose.

| page | adresse | à quoi elle sert |
| --- | --- | --- |
| serveur | `http://ADRESSE_PC:8000/` | score, terrain, référence, réglages, clips reçus |
| carte | `http://esp32cam.local/` | photo de cadrage, enregistrement brut, fichiers de la carte SD |

Tout ce qui se règle se règle sur la page du **serveur** ; la page servie
par la carte est le mode sans serveur et ne configure rien. Les garder
toutes les deux configurables invitait à régler des deux côtés sans
savoir lequel fait foi.

La carte est joignable par `http://esp32cam.local/` (nom mDNS, stable) ou
par l'adresse affichée sur le port série. `AP_ENABLED 1` ajoute son
propre Wi-Fi `ESP32-CAM` / `camera123` sur `http://192.168.4.1/`, au prix
du canal : l'AP suit alors le hotspot et éjecte le téléphone déjà
connecté.

## Ce qui déclenche un événement

Une *séquence* : du mouvement soutenu pendant au moins la durée minimale,
**puis** un retour au calme. Une personne immobile dans le champ ne
déclenche donc rien — la mesure est prise d'une image à la suivante, pas
contre la référence, donc une présence immobile lit 0 %.

## Enregistrement brut

Le bouton **Enregistrer la vidéo brute** écrit `/raw_NNNN.gray` sur la
carte SD : du gris 8 bits sans aucun codec, 768 ko/s, soit ~20 minutes par
gigaoctet. Sans compression, rejouer le fichier donne exactement les
chiffres calculés en direct — un JPEG déplacerait les valeurs de pixels et
décalerait tous les seuils.

```bash
python tools/replay.py raw_0001.gray --sweep
```

balaie une grille de réglages et compte les événements trouvés pour
chacun, ce qui évite de reflasher à chaque essai.

## Version simple : `esp32_cam_simple/`

Un croquis a part, independant du detecteur.

La carte monte **toujours son propre point d'acces** (`ESP32-CAM` /
`camera123`, page sur `http://192.168.4.1`) et, en parallele, essaie de
rejoindre le hotspot du telephone (`WIFI_SSID` / `WIFI_PASS` en haut du
fichier). Le point d'acces est ce qui garantit que la carte reste
joignable : un hotspot eteint, passe en 5 GHz ou en WPA3 ne rend plus la
carte muette. Les tentatives vers le hotspot s'arretent apres
`STA_MAX_TRY` essais - chacune fait scanner la radio, partagee avec l'AP
et la camera - et le lien *reessayer le hotspot* de la page les relance.
`AP_ONLY 1` coupe la station pour de bon.

En cas d'echec, le port serie dit pourquoi et liste les reseaux 2,4 GHz
visibles avec leur canal, leur niveau et leur chiffrement : un hotspot
absent de cette liste est en 5 GHz, masque ou eteint, et un hotspot en
WPA3 seul n'est pas joignable par un ESP32.

La page a deux boutons :

- **Prendre une image** -> `GET /shot`, un seul JPEG pris au moment du
  clic (converti juste pour le navigateur). Rien n'est diffuse en continu.
  La photo est en `PHOTO_SIZE` (640x480 par defaut) : elle ne passe pas
  par la carte SD, donc rien ne l'oblige a etre minuscule.
- **Demarrer / arreter l'enregistrement** -> `GET /record?on=1|0&fmt=jpg|rgb`,
  avec deux formats au choix (case a cocher sur la page) :

  | format | fichier | 5 s en QVGA | pour quoi |
  | --- | --- | --- | --- |
  | `jpg` (defaut) | `/rec_NNNN.mjpg` | ~300 ko | regarder, expedier |
  | `rgb` | `/rec_NNNN.rgb` | 3,75 Mo | rejouer un algorithme |

  Le brut garde les octets RGB565 du capteur tels quels : indispensable
  si les seuils doivent voir exactement les memes pixels qu'en direct.
  Le JPEG est converti en logiciel juste avant l'ecriture, le capteur ne
  change pas, et il divise la taille par douze.

  Le format se choisit au demarrage et nulle part ailleurs : en changer
  en cours d'enregistrement melangerait deux formats dans un fichier.

### Debit : c'est la carte SD qui decide

GPIO 4 porte Y2 de la camera et sert aussi de DATA1 au bus SD : le mode
4 bits est donc impossible sur cette carte, la SD reste en 1 bit. Le
debit exige vaut `largeur x hauteur x 2 x images/s`, et il faut au moins
30 % de marge - le reste du temps part dans le wifi et la camera.

Au demarrage le croquis ecrit ~4 Mo **par blocs de la taille d'une
image** (une carte SD est bien plus rapide par gros blocs : mesurer par
petits paquets la sous-estime), puis annonce le debit mesure, celui
qu'il faut, et combien d'images par seconde la carte tient reellement :

```
[SD] ecriture mesuree: 0.31 Mo/s par blocs de 37 ko
[SD] l'enregistrement demande 0.18 Mo/s
[SD] cette carte tient environ 6.4 images/s a 160x120
```

| carte | `REC_SIZE` / `REC_FPS` tenable |
| --- | --- |
| 0,3 Mo/s | QQVGA 160x120 @ 5 i/s (defaut) ou QVGA 320x240 @ 2 i/s |
| 1,0 Mo/s | QVGA 320x240 @ 5 i/s, ou QQVGA @ 20 i/s |

Seul l'enregistrement est concerne. Le capteur tourne en `PHOTO_SIZE` et
ne descend en `REC_SIZE` que pendant un enregistrement, puis remonte :
une carte lente degrade la video, jamais les photos. Le changement de
definition redemarre le pilote camera (~300 ms), le seul moyen en
RGB565 : le pilote fige a l'initialisation le nombre d'octets attendus
par image et refuse tout ce qui n'a pas cette taille.

Si la carte prend du retard en cours de route, les images en retard sont
**sautees** et comptees (`sautees` dans la page) au lieu de s'accumuler.

### Arrets automatiques

| cause | reglage |
| --- | --- |
| il reste moins de 20 Mo sur la carte | `SD_MIN_FREE_MB` |
| 30 minutes | `REC_MAX_MINUTES` |
| une ecriture SD echoue | - |

La place restante est suivie par soustraction depuis le debut de
l'enregistrement : interroger la FAT a chaque image ferait decrocher
l'ecriture. La raison de l'arret s'affiche sous les boutons.

### Resistance a la coupure

Les fichiers `.avi` de la version precedente n'etaient lisibles qu'apres
fermeture propre : l'index et les tailles n'etaient ecrits qu'a la fin,
donc une carte debranchee en pleine ecriture laissait un fichier
illisible, voire entierement a zero quand meme l'entete n'avait pas ete
vidangee.

Les deux formats de la version simple n'ont pas ce defaut :

- l'entete de 32 octets est complete et **vidangee des le depart** ;
- en `.rgb` toutes les images font la meme taille, donc leur nombre se
  deduit de la taille du fichier ; en `.mjpg` chaque image est precedee
  de sa longueur, ce qui permet de s'arreter net sur une image coupee -
  dans les deux cas rien a corriger a la fermeture ;
- vidange une fois par seconde, donc une coupure coute au pire une
  seconde d'images et jamais le fichier.

### Relire les fichiers sur le PC

Aucun lecteur ne connait le format `.rgb` : il faut le convertir, et
l'outil le fait sans rien a installer.

```bash
python tools/rgb_tool.py rec_0001.mjpg --avi video.avi # un .mjpg
python tools/rgb_tool.py rec_0001.rgb                # ce qu'il contient
python tools/rgb_tool.py rec_0001.rgb --avi video.avi # <-- lisible par VLC
python tools/rgb_tool.py rec_0001.rgb --png 0 50 100 # quelques images
python tools/rgb_tool.py rec_0001.rgb --mp4 out.mp4  # si ffmpeg est installe
```

`--avi` ecrit un AVI non compresse (BI_RGB 24 bits) : pas de codec, VLC
et Windows Media Player l'ouvrent directement. Il pese environ trois
fois le `.rgb`. Ajouter `--flip` pour un enregistrement fait avant le
correctif d'orientation.

L'outil se fie a la taille du fichier, pas a l'entete, et ignore une
derniere image incomplete : un enregistrement interrompu se relit sans
rien avoir a reparer.
