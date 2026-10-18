"""IA du rover : YOLOv8n sur le PC, video de la camera, ordres au rover.

    python rover_ia.py                 mode "voir" : detections seulement
    python rover_ia.py --mode suivre   suit une personne (demarre en PAUSE)

Le PC lit la video MJPEG de la carte camera (WiFi), detecte et suit les
personnes et objets avec YOLOv8n, et envoie au rover des ordres
"I vitesse rotation" par son WebSocket. Les reflexes du rover passent avant :
pas d'avance si un obstacle est a moins de 25 cm, arret automatique si l'IA
se tait 0,4 s ou se deconnecte.

Touches (fenetre video) :
    G       demarrer / reprendre le suivi (mode suivre)
    Espace  pause : le rover s'arrete
    C       choisir comme cible la personne la plus proche (la plus grande)
    Q, Echap  quitter (le rover s'arrete)
"""

import argparse
import json
import os
import socket
import threading
import time
import urllib.request

import cv2
import numpy as np
import websocket
from ultralytics import YOLO

DOSSIER_IA = os.path.join(os.environ.get("LOCALAPPDATA", "."), "RoverIA")
PERSONNE = 0                    # classe "person" de YOLO (COCO)
PERIODE_ORDRE = 0.1             # 10 ordres/s (le rover s'arrete apres 0,4 s sans ordre)


# ---------------------------------------------------------------- rover
class Rover:
    """WebSocket du rover : etat recu ([DATA], [CAM]) et ordres envoyes."""

    def __init__(self, hote):
        self.hote = hote
        self.etat = {}              # derniere trame [DATA]
        self.camera = {}            # dernier etat [CAM] relaye par le rover
        self.connecte = False
        self.ws = None
        self.verrou = threading.Lock()
        threading.Thread(target=self._boucle, daemon=True).start()

    def _boucle(self):
        while True:
            try:
                ip = socket.gethostbyname(self.hote)
                self.ws = websocket.create_connection(f"ws://{ip}:81/", timeout=5)
                self.connecte = True
                print(f"[IA] connecte au rover ({ip})")
                while True:
                    message = self.ws.recv()
                    for ligne in str(message).splitlines():
                        if ligne.startswith("[DATA]"):
                            self.etat = json.loads(ligne[6:])
                        elif ligne.startswith("[CAM]"):
                            self.camera = json.loads(ligne[5:])
            except Exception as e:
                if self.connecte:
                    print(f"[IA] rover perdu ({e}), reconnexion...")
                self.connecte = False
                time.sleep(2)

    def envoyer(self, texte):
        with self.verrou:
            if self.connecte:
                try:
                    self.ws.send(texte + "\n")
                except Exception:
                    pass

    def distance_avant(self):
        """mm, ou None si pas de mesure valide."""
        try:
            t = self.etat["tof"][0]
            return t["d"] if t["s"] == "ok" else None
        except (KeyError, IndexError, TypeError):
            return None


# ---------------------------------------------------------------- video
class Video:
    """Lit le flux MJPEG en continu et garde seulement l'image la plus recente
    (sinon les images s'accumulent et l'IA prend du retard)."""

    def __init__(self, url):
        self.url = url
        self.image = None
        self.numero = 0
        threading.Thread(target=self._boucle, daemon=True).start()

    # Lecteur MJPEG maison : le module video d'OpenCV (FFmpeg) reste bloque
    # sur le flux de la camera. On lit le HTTP et on decoupe chaque image
    # JPEG entre ses marqueurs de debut (FF D8) et de fin (FF D9).
    def _boucle(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=10) as flux:
                    print(f"[IA] video : {self.url}")
                    tampon = b""
                    while True:
                        morceau = flux.read(16384)
                        if not morceau:
                            break
                        tampon += morceau
                        fin = tampon.rfind(b"\xff\xd9")
                        debut = tampon.rfind(b"\xff\xd8", 0, fin) if fin > 0 else -1
                        if debut >= 0:
                            jpeg = tampon[debut:fin + 2]
                            tampon = tampon[fin + 2:]
                            image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                            if image is not None:
                                self.image = image          # la plus recente seulement
                                self.numero += 1
                        elif len(tampon) > 2_000_000:
                            tampon = b""
            except Exception as e:
                print(f"[IA] video indisponible ({e}), nouvel essai...")
            time.sleep(1)


ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


# ---------------------------------------------------------------- suivi
class Suivi:
    """Garde la personne cible au centre et a la bonne distance."""

    def __init__(self, args):
        self.args = args
        self.cible = None           # identifiant de suivi YOLO
        self.derniere_vue = 0.0
        self.actif = False          # False = pause

    def choisir(self, personnes):
        if personnes:
            self.cible = max(personnes, key=lambda p: p["h"])["id"]
            self.derniere_vue = time.time()
            print(f"[IA] cible : personne #{self.cible}")

    def ordre(self, personnes, largeur, hauteur, distance_avant):
        """(vitesse, rotation, explication) ; (0, 0) si rien a faire."""
        if not self.actif:
            return 0, 0, "PAUSE (G pour suivre)"
        if self.cible is None:
            self.choisir(personnes)
            if self.cible is None:
                return 0, 0, "aucune personne"
        p = next((p for p in personnes if p["id"] == self.cible), None)
        if p is None:
            if time.time() - self.derniere_vue > 1.0:
                return 0, 0, f"cible #{self.cible} perdue : attente"
            return 0, 0, "cible masquee"
        self.derniere_vue = time.time()

        # direction : ecart horizontal du centre de la personne (-1 .. 1)
        ecart = (p["cx"] - largeur / 2) / (largeur / 2)
        rotation = int(-ecart * self.args.rotation_max)       # a droite = rotation negative
        if abs(ecart) < 0.08:
            rotation = 0

        # distance : capteur avant si la personne est droit devant, sinon
        # taille de la personne dans l'image
        if distance_avant is not None and abs(ecart) < 0.25 and distance_avant < 3500:
            erreur = (distance_avant - self.args.distance * 1000) / 1000     # m
            source = f"capteur {distance_avant} mm"
        else:
            erreur = (self.args.hauteur_cible - p["h"] / hauteur) * 3        # ~ m
            source = f"taille {p['h'] / hauteur:.2f}"
        vitesse = int(max(-self.args.vitesse_max / 2, min(self.args.vitesse_max, erreur * 200)))
        if abs(erreur) < 0.1:
            vitesse = 0
        if abs(ecart) > 0.5:
            vitesse = min(vitesse, 0)                           # tourner d'abord
        return vitesse, rotation, f"suit #{self.cible} ({source})"


# ---------------------------------------------------------------- principal
def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--mode", choices=["voir", "suivre"], default="voir")
    a.add_argument("--rover", default="rover.local")
    a.add_argument("--camera", help="adresse de la camera (sinon donnee par le rover)")
    a.add_argument("--modele", default=os.path.join(DOSSIER_IA, "yolov8n.pt"))
    a.add_argument("--confiance", type=float, default=0.45)
    a.add_argument("--taille", type=int, default=416, help="taille d'image pour YOLO (plus petit = plus rapide)")
    a.add_argument("--distance", type=float, default=1.0, help="distance de suivi (m)")
    a.add_argument("--hauteur-cible", type=float, default=0.6, help="part de l'image occupee par la personne a cette distance")
    a.add_argument("--vitesse-max", type=int, default=110)
    a.add_argument("--rotation-max", type=int, default=90)
    args = a.parse_args()

    os.makedirs(DOSSIER_IA, exist_ok=True)
    print(f"[IA] modele {args.modele} (telecharge au premier lancement)")
    modele = YOLO(args.modele)

    rover = Rover(args.rover)
    url = f"http://{args.camera}:81/stream" if args.camera else None
    attente = time.time() + 10
    while url is None and time.time() < attente:
        if rover.camera.get("flux"):
            url = rover.camera["flux"]
        time.sleep(0.2)
    if url is None:
        raise SystemExit("Camera introuvable : rover non connecte ? Preciser --camera <ip>")
    video = Video(url)
    suivi = Suivi(args)
    commande = args.mode == "suivre"
    fenetre = "Rover IA - " + args.mode
    dernier_ordre, dernier_numero, t_fps, n_fps, fps = 0.0, -1, time.time(), 0, 0.0
    print(f"[IA] mode {args.mode}" + (" : PAUSE, appuyer sur G pour suivre" if commande else " (aucun ordre moteur)"))

    try:
        while True:
            image = video.image
            if image is None or video.numero == dernier_numero:
                if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                continue
            dernier_numero = video.numero
            rotation_cam = int(rover.camera.get("rotation", 0) or 0)
            if rotation_cam in ROTATIONS:
                image = cv2.rotate(image, ROTATIONS[rotation_cam])
            h, w = image.shape[:2]

            resultat = modele.track(image, persist=True, conf=args.confiance, imgsz=args.taille,
                                    verbose=False, tracker="bytetrack.yaml")[0]
            personnes, objets = [], []
            if resultat.boxes is not None and resultat.boxes.id is not None:
                for boite, ident, classe in zip(resultat.boxes.xyxy.tolist(),
                                                resultat.boxes.id.int().tolist(),
                                                resultat.boxes.cls.int().tolist()):
                    x1, y1, x2, y2 = boite
                    d = {"id": ident, "cx": (x1 + x2) / 2, "h": y2 - y1, "nom": modele.names[classe]}
                    (personnes if classe == PERSONNE else objets).append(d)
            vue = resultat.plot()

            # ordres au rover (mode suivre seulement)
            texte = "VOIR : aucun ordre moteur"
            if commande:
                vitesse, rotation, texte = suivi.ordre(personnes, w, h, rover.distance_avant())
                if time.time() - dernier_ordre >= PERIODE_ORDRE:
                    if suivi.actif:
                        rover.envoyer(f"I {vitesse} {rotation}")
                    dernier_ordre = time.time()
                texte += f"  -> v={vitesse} r={rotation}"

            # affichage
            n_fps += 1
            if time.time() - t_fps >= 1:
                fps, n_fps, t_fps = n_fps / (time.time() - t_fps), 0, time.time()
            d_av = rover.distance_avant()
            infos = [f"{fps:.1f} img/s  rover {'OK' if rover.connecte else 'NON CONNECTE'}"
                     f"  avant {d_av if d_av is not None else '--'} mm",
                     texte,
                     "objets : " + (", ".join(sorted({o['nom'] for o in objets})) or "-")]
            for i, t in enumerate(infos):
                cv2.putText(vue, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(vue, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.imshow(fenetre, vue)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):
                break
            if commande and touche == ord("g"):
                suivi.actif = True
                print("[IA] suivi ACTIF")
            if commande and touche == ord(" "):
                suivi.actif = False
                rover.envoyer("s")
                print("[IA] PAUSE : rover arrete")
            if touche == ord("c"):
                suivi.choisir(personnes)
    finally:
        if commande:
            rover.envoyer("s")          # le rover s'arrete en quittant
        cv2.destroyAllWindows()
        print("[IA] termine")


if __name__ == "__main__":
    main()
