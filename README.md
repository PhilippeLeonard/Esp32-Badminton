# Esp32-Badminton

Détecteur de changement sur Freenove ESP32-WROVER CAM. La carte compare
chaque image à une référence, n'envoie qu'un **pourcentage**, et ne
transmet des images que lorsqu'un événement se termine.

## Disposition du dépôt

```
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
