# Prompt pour Claude : faire évoluer l'IA YOLO du rover

Copier le bloc ci-dessous dans Claude Code (ouvert dans le dossier du projet), puis
remplacer la dernière ligne par ce que l'on veut obtenir.

```text
Projet PlatformIO « Experimental_bot » : rover ESP32-S3 + carte caméra ESP32-S3 (OV2640),
IA YOLOv8n sur le PC. Lis d'abord docs/IA.md, docs/Architecture.md et
docs/architecture_rover.html, puis ia/rover_ia.py.

Fonctionnement actuel :
- ia/rover_ia.py lit la vidéo MJPEG de la caméra sur le port 82 (le port 81 est pour la
  page), détecte avec YOLOv8n + ByteTrack, et se connecte au WebSocket du rover
  (ws://rover.local:81/).
- Elle s'annonce par « [IA]{...} » ; ses trames « [IA]{"det":[[x,y,l,h,"nom",conf,id],...],
  "img":[l,h],"ips":..,"conf":..,"classes":[..],"affichage":..,"suivi":..,"classe":..,
  "cible":..,"texte":..,"v":..,"r":..} » (5/s, coordonnées 0..1 dans l'image remise
  droite) sont relayées par le rover à la page web (web/rover_moniteur.html), qui dessine
  les cadres sur la vidéo.
- La page et les missions lui envoient « [POURIA] conf 0.4 | classes person,cup |
  affichage 1/0 | suivre <classe> | stop » (noms YOLO, espaces remplacés par « _ »).
- Elle ne pilote que pendant une mission « suivre » (M suivre <classe> sur le rover, ou
  touche G) : ordres « I vitesse rotation » jusqu’à 10 fois/s. Le rover garde ses réflexes : pas
  d'avance si obstacle < 25 cm, arrêt si l'IA se tait 0,4 s ou se déconnecte.
- Environnement Python : %LOCALAPPDATA%\RoverIA\venv (hors OneDrive), installé par
  ia/installer.ps1. Tâches VS Code dans .vscode/tasks.json.

Règles à respecter :
- Ne jamais affaiblir les réflexes de sécurité du rover ni le STOP ; l'IA ne pilote que
  pendant une mission de suivi.
- Ne pas envoyer d'ordre moteur au rover pour tester : je fais moi-même les essais qui
  roulent. Pour tester sans risque : --vitesse-max 0 --rotation-max 0.
- Garder le protocole [IA] / [POURIA] compatible avec la page, ou mettre à jour la page
  et docs/IA.md en même temps.
- Secrets dans include/secrets.h et secrets.ini : jamais dans git (dépôt public).
- Mettre à jour docs/IA.md, docs/architecture_rover.html (+ PNG) si le fonctionnement
  change, puis commit et push.
- Discuter de l'architecture avec moi avant un gros changement.

Ce que je veux maintenant : <décrire ici>
```

## Idées d'évolution (à coller à la place de la dernière ligne)

- **Recherche de la cible** : quand la personne est perdue, tourner lentement sur place
  (± 90°) pour la retrouver, au lieu d'attendre.
- **Support caméra 2 axes** (quand il sera monté) : la caméra suit la cible avec les servos
  pan/tilt (PCA9685 0x40, canaux 1 et 2), le rover ne tourne que si l'angle dépasse ±30°.
- **Obstacles vus par la caméra** : signaler au rover les obstacles bas que les capteurs
  ToF ne voient pas (pieds de chaise, câbles), pour la navigation autonome.
- **Modèle plus précis** : essayer yolov8s ou YOLO11n, comparer images/s et précision.
- **Modèle entraîné** sur mes propres objets (photos prises avec la caméra du rover,
  dossier /photos de la carte TF).
- **Retour à la base guidé par la caméra** : reconnaître un repère visuel (marqueur ArUco)
  posé à la base.
- **Réglages gardés** : mémoriser sur le PC les derniers réglages de la page (confiance,
  objets affichés) pour les retrouver au prochain lancement.
- **Statistiques dans le journal** : envoyer au rover un résumé des détections pour qu'il
  soit écrit dans le journal de navigation de la carte TF.
