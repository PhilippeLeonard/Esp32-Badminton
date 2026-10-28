# Esp32-Badminton

Détecteur de changement sur Freenove ESP32-WROVER CAM. La carte compare
chaque image à une référence, n'envoie qu'un **pourcentage**, et ne
transmet des images que lorsqu'un événement se termine.

## Disposition du dépôt

```
esp32_cam_simple/
    esp32_cam_simple.ino     la version simple (photo + enregistrement)
esp32_cam_recorder/
    esp32_cam_recorder.ino   le firmware  <-- ouvrir CE dossier dans l'IDE
    app_page.h               généré, ne pas éditer à la main
phone_app.html               l'interface web (la source à modifier)
make_page.py                 phone_app.html  ->  esp32_cam_recorder/app_page.h
tools/replay.py              rejoue la détection sur un enregistrement .gray
```

L'IDE Arduino n'ouvre un croquis que s'il est dans un dossier portant son
nom. C'est pourquoi le `.ino` est dans `esp32_cam_recorder/` et non à la
racine : ouvert depuis la racine, l'IDE en fabrique une copie ailleurs, et
on finit par éditer un fichier pendant qu'on en flashe un autre.

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

## Utilisation

1. Ouvrir `http://192.168.4.1/` (téléphone connecté au Wi-Fi `ESP32-CAM`,
   mot de passe `camera123`), ou l'adresse affichée sur le port série si
   la carte a rejoint le hotspot.
2. Scène au repos → **Prendre la référence** : l'exposition se verrouille
   et huit images sont moyennées.
3. **Voir le masque** pour régler la sensibilité par pixel.
4. Un événement se déclenche sur une *séquence* : du mouvement soutenu
   pendant au moins la durée minimale, **puis** un retour au calme. Une
   personne immobile dans le champ ne déclenche donc rien.

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

Un croquis a part, independant du detecteur. La carte rejoint le hotspot
du telephone (`WIFI_SSID` / `WIFI_PASS` en haut du fichier) et sert une
page a deux boutons sur `http://esp32cam.local` ou sur l'adresse affichee
au port serie :

- **Prendre une image** -> `GET /shot`, un seul JPEG pris au moment du
  clic (converti juste pour le navigateur). Rien n'est diffuse en continu.
- **Demarrer / arreter l'enregistrement** -> `GET /record?on=1|0`, ecrit
  `/rec_NNNN.rgb` sur la carte SD : **couleur, aucune compression**, les
  octets RGB565 sortis du capteur tels quels.

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

| carte | reglage tenable |
| --- | --- |
| 0,3 Mo/s | QQVGA 160x120 @ 5 i/s (defaut) ou QVGA 320x240 @ 2 i/s |
| 1,0 Mo/s | QVGA 320x240 @ 5 i/s, ou QQVGA @ 20 i/s |

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

Le format `.rgb` n'a pas ce defaut :

- l'entete de 32 octets est complete et **vidangee des le depart** ;
- toutes les images font la meme taille, donc leur nombre se deduit de la
  taille du fichier - rien a corriger a la fermeture ;
- vidange une fois par seconde, donc une coupure coute au pire une
  seconde d'images et jamais le fichier.

### Relire les fichiers sur le PC

```bash
python tools/rgb_tool.py rec_0001.rgb                # ce qu'il contient
python tools/rgb_tool.py rec_0001.rgb --png 0 50 100 # quelques images
python tools/rgb_tool.py rec_0001.rgb --mp4 out.mp4  # si ffmpeg est installe
```

L'outil se fie a la taille du fichier, pas a l'entete, et ignore une
derniere image incomplete : un enregistrement interrompu se relit sans
rien avoir a reparer.
