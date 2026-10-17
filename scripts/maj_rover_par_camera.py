"""Programme le rover a travers la carte camera (un seul cable : celui de la camera).

Utilise par l'environnement PlatformIO test_capteurs_via_camera :
    pio run -e test_capteurs_via_camera -t upload
ou directement :
    python maj_rover_par_camera.py chemin/firmware.bin [--port COMx]

Le programme part sur le cable USB de la camera (460800 bauds), la camera le
verifie (CRC32) puis l'envoie au rover par le WiFi (POST /maj). Le moniteur
serie de la camera doit etre ferme pendant l'envoi.
Protocole : voir src/camera/RelaisMaj.h.
"""

import argparse
import os
import sys
import time
import zlib

import serial
from serial.tools import list_ports

VIDPID_CAMERA = (0x1A86, 0x7523)       # puce CH340 de la carte camera
VITESSE = 460800
TAILLE_BLOC = 4096


def trouver_port():
    for p in list_ports.comports():
        if (p.vid, p.pid) == VIDPID_CAMERA:
            return p.device
    return None


def lire_ligne(s, attente):
    """Ligne suivante commencant par [MAJ] ou [PONT] (les autres sont affichees)."""
    fin = time.time() + attente
    while time.time() < fin:
        brut = s.readline()
        if not brut:
            continue
        ligne = brut.decode("utf-8", "replace").strip()
        if ligne.startswith("[MAJ]") or ligne.startswith("[PONT]"):
            return ligne
    return None


def attendre_camera(s):
    """La camera peut redemarrer a l'ouverture du port : on attend qu'elle reponde."""
    fin = time.time() + 25
    while time.time() < fin:
        s.write(b"\n!PING\n")
        ligne = lire_ligne(s, 1.0)
        if ligne == "[PONT] PONG":
            return True
    return False


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("firmware")
    a.add_argument("--port", help="port de la camera (sinon trouve par sa puce CH340)")
    args = a.parse_args()

    donnees = open(args.firmware, "rb").read()
    crc = zlib.crc32(donnees) & 0xFFFFFFFF
    port = args.port or trouver_port()
    if not port:
        sys.exit("Carte camera introuvable (puce CH340 1A86:7523). Est-elle branchee ?")

    print(f"Programme du rover : {os.path.basename(args.firmware)}, {len(donnees)} octets, CRC {crc:08x}")
    print(f"Envoi par la camera sur {port}...")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, VITESSE, 0.5
    s.dtr = False                       # ne pas redemarrer la camera
    s.rts = False
    try:
        s.open()
    except serial.SerialException as e:
        sys.exit(f"Port {port} indisponible ({e}). Fermer le moniteur serie de la camera.")

    try:
        if not attendre_camera(s):
            sys.exit("La camera ne repond pas (programme camera a jour ? vitesse 460800 ?)")
        s.write(f"!MAJROVER {len(donnees)} {crc:08x}\n".encode())
        ligne = lire_ligne(s, 5)
        if ligne != "[MAJ] PRET":
            sys.exit(f"La camera refuse : {ligne}")

        debut = time.time()
        for pos in range(0, len(donnees), TAILLE_BLOC):
            bloc = donnees[pos:pos + TAILLE_BLOC]
            s.write(bloc)
            ligne = lire_ligne(s, 6)
            if not ligne or not ligne.startswith("[MAJ] ACK"):
                sys.exit(f"\nTransfert interrompu : {ligne}")
            fait = pos + len(bloc)
            print(f"\r  cable -> camera : {fait * 100 // len(donnees):3d} %", end="", flush=True)
        print(f"  ({time.time() - debut:.1f} s)")

        # verification, puis envoi WiFi au rover et sa reponse
        while True:
            ligne = lire_ligne(s, 120)
            if ligne is None:
                sys.exit("Pas de reponse de la camera (delai depasse)")
            print("  " + ligne)
            if ligne.startswith("[MAJ] OK"):
                print("Rover mis a jour : il redemarre.")
                return
            if ligne.startswith("[MAJ] ERREUR"):
                sys.exit(1)
    finally:
        s.close()


if __name__ == "__main__":
    main()
