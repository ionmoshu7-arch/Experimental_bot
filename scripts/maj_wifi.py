"""Envoie un programme a une carte par le WiFi (POST http://<carte>/maj).

Utilise par les environnements PlatformIO test_capteurs_ota et camera_ota :
    pio run -e camera_ota -t upload
C'est le PC qui se connecte a la carte : le Pare-feu Windows ne bloque pas,
contrairement a l'OTA classique (espota). Le mot de passe vient de secrets.ini
(upload_flags = --auth=...).
"""

import argparse
import socket
import sys
import time
import urllib.error
import urllib.request


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("hote", help="rover.local, camera.local ou une adresse IP")
    a.add_argument("firmware")
    a.add_argument("--auth", required=True, help="mot de passe OTA")
    args = a.parse_args()

    try:
        ip = socket.gethostbyname(args.hote)
    except OSError:
        sys.exit(f"{args.hote} introuvable sur le reseau (essayer l'adresse IP)")

    donnees = open(args.firmware, "rb").read()
    print(f"Envoi de {len(donnees)} octets a {args.hote} ({ip})...")
    requete = urllib.request.Request(
        f"http://{ip}/maj", data=donnees, method="POST",
        headers={"Content-Type": "application/octet-stream", "X-Mot-De-Passe": args.auth})
    debut = time.time()
    try:
        with urllib.request.urlopen(requete, timeout=120) as r:
            print(f"{r.read().decode(errors='replace')} ({time.time() - debut:.1f} s)")
    except urllib.error.HTTPError as e:
        sys.exit(f"Refuse par la carte (HTTP {e.code}) : {e.read().decode(errors='replace')}")
    except (urllib.error.URLError, OSError) as e:
        sys.exit(f"Envoi impossible : {e}")


if __name__ == "__main__":
    main()
