#!/bin/sh
# macOS : double-clic depuis le Finder. Le repertoire courant du Finder
# n'est pas celui du script, d'ou le cd.
cd "$(dirname "$0")" || exit 1
for py in python3 python; do
  if command -v "$py" >/dev/null 2>&1; then
    exec "$py" demarrer.py "$@"
  fi
done
echo "Python 3 est introuvable. Installez-le, puis relancez ce fichier."
read -r _
exit 1
