"""IA du rover : YOLOv8n sur le PC, video de la camera, ordres au rover.

    python rover_ia.py                 fenetre video + detections
    python rover_ia.py --service       sans fenetre, pilotee par la page web
    python rover_ia.py --mode suivre   suit une personne des le lancement (en PAUSE, G)

Le PC lit la video MJPEG de la carte camera (WiFi), detecte et suit les
objets avec YOLOv8n + ByteTrack, et :
  - envoie ses detections a la page (cadres sur la video), 5 fois/s :
        [IA]{"det":[[x,y,l,h,"nom",confiance,id],...],"img":[l,h],...}
    (coordonnees 0..1 dans l'image remise droite) ;
  - recoit les reglages de la page : [POURIA] conf 0.5 | classes person,chair
    | affichage 1/0 | suivre <classe> | stop ;
  - pendant une mission "suivre" (bouton de la page, ou G dans la fenetre),
    envoie au rover des ordres "I vitesse rotation", un par image (10/s au plus).
Les reflexes du rover passent avant : pas d'avance si un obstacle est a
moins de 25 cm, arret si l'IA se tait 0,4 s ou se deconnecte.

Touches (fenetre video) :
    G       suivre (personne, ou la classe demandee par la page)
    Espace  pause : le rover s'arrete
    C       choisir comme cible le plus grand objet de la classe suivie
    Q, Echap  quitter (le rover s'arrete)
"""

import argparse
import json
import os
import queue
import socket
import threading
import time
import urllib.request

import cv2
import numpy as np
import websocket
from ultralytics import YOLO

DOSSIER_IA = os.path.join(os.environ.get("LOCALAPPDATA", "."), "RoverIA")
PERIODE_ORDRE = 0.1             # 10 ordres/s (le rover s'arrete apres 0,4 s sans ordre)
PERIODE_DETECTIONS = 0.2        # 5 envois de detections/s vers la page
ROTATIONS = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}


# ---------------------------------------------------------------- rover
class Rover:
    """WebSocket du rover : etat recu ([DATA], [CAM]), reglages de la page
    ([POURIA]) et messages envoyes (ordres I, detections [IA])."""

    def __init__(self, hote):
        self.hote = hote
        self.etat = {}              # derniere trame [DATA]
        self.camera = {}            # dernier etat [CAM] relaye par le rover
        self.reglages = queue.Queue()   # lignes "[POURIA] ..." recues
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
                self.envoyer('[IA]{"etat":"pret"}')     # le rover nous reconnait comme IA
                while True:
                    message = self.ws.recv()
                    for ligne in str(message).splitlines():
                        try:
                            if ligne.startswith("[DATA]"):
                                self.etat = json.loads(ligne[6:])
                            elif ligne.startswith("[CAM]"):
                                self.camera = json.loads(ligne[5:])
                            elif ligne.startswith("[POURIA]"):
                                self.reglages.put(ligne[8:].strip())
                        except ValueError:
                            pass        # trame abimee : on l'ignore, sans couper la liaison
            except Exception as e:
                if self.connecte:
                    print(f"[IA] rover perdu ({e}), reconnexion...")
                self.connecte = False
                with self.verrou:
                    try:
                        if self.ws:
                            self.ws.close()     # libere la place sur le rover (8 au plus)
                    except Exception:
                        pass
                time.sleep(2)

    def envoyer(self, texte):
        with self.verrou:
            if self.connecte:
                try:
                    self.ws.send(texte if texte.startswith("[IA]") else texte + "\n")
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
    """Lit le flux MJPEG en continu et garde seulement l'image la plus recente.
    Lecteur maison : le module video d'OpenCV (FFmpeg) reste bloque sur le flux
    de la camera. On decoupe chaque JPEG entre ses marqueurs FF D8 et FF D9."""

    def __init__(self, url):
        self.url = url
        self.image = None
        self.numero = 0
        threading.Thread(target=self._boucle, daemon=True).start()

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
                                self.image = image
                                self.numero += 1
                        elif len(tampon) > 2_000_000:
                            tampon = b""
            except Exception as e:
                print(f"[IA] video indisponible ({e}), nouvel essai...")
            time.sleep(1)


# ---------------------------------------------------------------- suivi
class Suivi:
    """Garde l'objet cible (personne ou autre classe) au centre et a la bonne
    distance. La camera regarde droit devant : direction = cap du rover."""

    def __init__(self, args):
        self.args = args
        self.classe = "person"
        self.cible = None           # identifiant de suivi YOLO
        self.derniere_vue = 0.0
        self.actif = False

    def demarrer(self, classe):
        self.classe = classe or "person"
        self.cible = None
        self.actif = True
        print(f"[IA] suivi ACTIF : {self.classe}")

    def arreter(self):
        self.actif = False
        self.cible = None

    def choisir(self, objets):
        candidats = [o for o in objets if o["nom"] == self.classe]
        if candidats:
            self.cible = max(candidats, key=lambda o: o["h"])["id"]
            self.derniere_vue = time.time()
            print(f"[IA] cible : {self.classe} #{self.cible}")

    def ordre(self, objets, largeur, hauteur, distance_avant):
        """(vitesse, rotation, explication)."""
        if not self.actif:
            return 0, 0, "pas de suivi (mission Suivre, ou G)"
        if self.cible is None:
            self.choisir(objets)
            if self.cible is None:
                return 0, 0, f"aucun(e) {self.classe} en vue"
        o = next((o for o in objets if o["id"] == self.cible), None)
        if o is None:
            if time.time() - self.derniere_vue > 1.0:
                self.cible = None           # on reprendra le plus grand visible
                return 0, 0, f"{self.classe} perdu(e) : attente"
            return 0, 0, "cible masquee"
        self.derniere_vue = time.time()

        ecart = (o["cx"] - largeur / 2) / (largeur / 2)        # -1 gauche .. 1 droite
        rotation = 0 if abs(ecart) < 0.08 else int(-ecart * self.args.rotation_max)
        if distance_avant is not None and abs(ecart) < 0.25 and distance_avant < 3500:
            erreur = (distance_avant - self.args.distance * 1000) / 1000
            source = f"capteur {distance_avant} mm"
        else:
            erreur = (self.args.hauteur_cible - o["h"] / hauteur) * 3
            source = f"taille {o['h'] / hauteur:.2f}"
        vitesse = 0 if abs(erreur) < 0.1 else int(max(-self.args.vitesse_max / 2,
                                                       min(self.args.vitesse_max, erreur * 200)))
        if abs(ecart) > 0.5:
            vitesse = min(vitesse, 0)                           # tourner d'abord
        return vitesse, rotation, f"suit {self.classe} #{self.cible} ({source})"


# ---------------------------------------------------------------- principal
def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--mode", choices=["voir", "suivre"], default="voir")
    a.add_argument("--service", action="store_true", help="sans fenetre (pilotee par la page)")
    a.add_argument("--rover", default="rover.local")
    a.add_argument("--camera", help="adresse de la camera (sinon donnee par le rover)")
    a.add_argument("--modele", default=os.path.join(DOSSIER_IA, "yolov8n.pt"))
    a.add_argument("--confiance", type=float, default=0.45)
    a.add_argument("--taille", type=int, default=416, help="taille d'image pour YOLO (plus petit = plus rapide)")
    a.add_argument("--distance", type=float, default=1.0, help="distance de suivi (m)")
    a.add_argument("--hauteur-cible", type=float, default=0.6, help="part de l'image occupee par la cible a cette distance")
    a.add_argument("--vitesse-max", type=int, default=110)
    a.add_argument("--rotation-max", type=int, default=90)
    args = a.parse_args()

    os.makedirs(DOSSIER_IA, exist_ok=True)
    print(f"[IA] modele {args.modele} (telecharge au premier lancement)")
    modele = YOLO(args.modele)
    noms_classes = set(modele.names.values())

    rover = Rover(args.rover)
    # flux 82 reserve a l'IA : la page garde le flux 81 (un seul ecran par flux)
    url = f"http://{args.camera}:82/stream" if args.camera else None
    attente = time.time() + 15
    while url is None and time.time() < attente:
        if rover.camera.get("fluxIA"):
            url = rover.camera["fluxIA"]
        elif rover.camera.get("flux") and time.time() > attente - 10:
            url = rover.camera["flux"]          # ancienne camera : un seul flux
        time.sleep(0.2)
    if url is None:
        raise SystemExit("Camera introuvable : rover non connecte ? Preciser --camera <ip>")
    video = Video(url)
    suivi = Suivi(args)
    reglage = {"conf": args.confiance, "classes": set(), "affichage": True}
    fenetre = None if args.service else "Rover IA"
    if args.mode == "suivre":
        suivi.classe = "person"
        print("[IA] mode suivre : PAUSE, appuyer sur G")
    dernier_ordre = dernier_envoi = 0.0
    dernier_numero, t_fps, n_fps, fps = -1, time.time(), 0, 0.0
    texte = ""

    try:
        while True:
            # ---- reglages venus de la page
            while not rover.reglages.empty():
                # noms a plusieurs mots envoyes avec "_" : sports_ball -> sports ball
                cmd = [c.replace("_", " ") for c in rover.reglages.get().split()]
                if not cmd:
                    continue
                if cmd[0] == "conf" and len(cmd) > 1:
                    try:
                        reglage["conf"] = max(0.05, min(0.95, float(cmd[1])))
                    except ValueError:
                        print(f"[IA] reglage ignore (nombre attendu) : {' '.join(cmd)}")
                        continue
                elif cmd[0] == "classes":
                    reglage["classes"] = {c for c in (cmd[1].split(",") if len(cmd) > 1 else []) if c in noms_classes}
                elif cmd[0] == "affichage" and len(cmd) > 1:
                    reglage["affichage"] = cmd[1] == "1"
                elif cmd[0] == "suivre":
                    suivi.demarrer(cmd[1] if len(cmd) > 1 and cmd[1] in noms_classes else "person")
                elif cmd[0] == "stop":
                    suivi.arreter()
                    rover.envoyer("I 0 0")
                print(f"[IA] reglage page : {' '.join(cmd)}")

            image = video.image
            if image is None or video.numero == dernier_numero:
                if fenetre and cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                time.sleep(0.005)
                continue
            dernier_numero = video.numero
            rotation_cam = int(rover.camera.get("rotation", 0) or 0)
            if rotation_cam in ROTATIONS:
                image = cv2.rotate(image, ROTATIONS[rotation_cam])
            h, w = image.shape[:2]

            resultat = modele.track(image, persist=True, conf=reglage["conf"], imgsz=args.taille,
                                    verbose=False, tracker="bytetrack.yaml")[0]
            objets = []
            if resultat.boxes is not None and len(resultat.boxes):
                ids = resultat.boxes.id.int().tolist() if resultat.boxes.id is not None else [-1] * len(resultat.boxes)
                for boite, ident, classe, conf in zip(resultat.boxes.xyxy.tolist(), ids,
                                                       resultat.boxes.cls.int().tolist(),
                                                       resultat.boxes.conf.tolist()):
                    nom = modele.names[classe]
                    if reglage["classes"] and nom not in reglage["classes"] and nom != suivi.classe:
                        continue
                    x1, y1, x2, y2 = boite
                    objets.append({"id": ident, "nom": nom, "conf": conf, "x1": x1, "y1": y1,
                                   "cx": (x1 + x2) / 2, "l": x2 - x1, "h": y2 - y1})

            # ---- suivi : ordres au rover
            vitesse, rotation, texte = suivi.ordre(objets, w, h, rover.distance_avant())
            if suivi.actif and time.time() - dernier_ordre >= PERIODE_ORDRE:
                rover.envoyer(f"I {vitesse} {rotation}")
                dernier_ordre = time.time()

            # ---- detections vers la page
            n_fps += 1
            if time.time() - t_fps >= 1:
                fps, n_fps, t_fps = n_fps / (time.time() - t_fps), 0, time.time()
            if time.time() - dernier_envoi >= PERIODE_DETECTIONS:
                dernier_envoi = time.time()
                det = [[round(o["x1"] / w, 3), round(o["y1"] / h, 3), round(o["l"] / w, 3),
                        round(o["h"] / h, 3), o["nom"], round(o["conf"], 2), o["id"]]
                       for o in objets] if reglage["affichage"] else []
                rover.envoyer("[IA]" + json.dumps({
                    "det": det, "img": [w, h], "ips": round(fps, 1), "conf": reglage["conf"],
                    "classes": sorted(reglage["classes"]), "affichage": reglage["affichage"],
                    "suivi": suivi.actif, "classe": suivi.classe, "cible": suivi.cible,
                    "texte": texte, "v": vitesse, "r": rotation}, separators=(",", ":")))

            # ---- fenetre (pas en mode service)
            if fenetre:
                vue = resultat.plot()
                d_av = rover.distance_avant()
                infos = [f"{fps:.1f} img/s  rover {'OK' if rover.connecte else 'NON CONNECTE'}"
                         f"  avant {d_av if d_av is not None else '--'} mm",
                         texte + (f"  -> v={vitesse} r={rotation}" if suivi.actif else ""),
                         "objets : " + (", ".join(sorted({o['nom'] for o in objets})) or "-")]
                for i, t in enumerate(infos):
                    cv2.putText(vue, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                    cv2.putText(vue, t, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.imshow(fenetre, vue)
                touche = cv2.waitKey(1) & 0xFF
                if touche in (ord("q"), 27):
                    break
                if touche == ord("g"):
                    suivi.demarrer(suivi.classe)
                if touche == ord(" "):
                    suivi.arreter()
                    rover.envoyer("s")
                    print("[IA] PAUSE : rover arrete")
                if touche == ord("c"):
                    suivi.choisir(objets)
    except KeyboardInterrupt:
        pass
    finally:
        if suivi.actif:
            rover.envoyer("s")          # le rover s'arrete en quittant
        if fenetre:
            cv2.destroyAllWindows()
        print("[IA] termine")


if __name__ == "__main__":
    main()
