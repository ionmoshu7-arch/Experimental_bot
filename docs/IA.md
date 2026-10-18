# IA du rover : YOLOv8n sur le PC

```
 Carte caméra ──WiFi, vidéo MJPEG──► PC : ia/rover_ia.py (YOLOv8n + suivi ByteTrack)
                                      │
 Rover ◄──────── WiFi, WebSocket ─────┘   ordres « I vitesse rotation », 10 par seconde
       └────────► état du rover ([DATA]) et adresse de la caméra ([CAM])
```

## Lancer

Dans VS Code, menu **Terminal > Exécuter la tâche…** :

| Tâche | Effet |
|---|---|
| **IA rover : voir** | fenêtre vidéo avec les détections (personnes, chaises, sacs… ; 80 sortes d'objets). **Aucun ordre aux moteurs.** |
| **IA rover : suivre une personne** | le rover suit la personne choisie ; démarre **en pause** |
| **IA rover : installer / mettre à jour** | (ré)installe Python 3.12, PyTorch, YOLO, OpenCV |

Au premier lancement, le modèle `yolov8n.pt` (environ 6 Mo) se télécharge tout seul.

Touches, dans la fenêtre vidéo :

| Touche | Effet |
|---|---|
| **G** | démarrer ou reprendre le suivi |
| **Espace** | pause : le rover s'arrête |
| **C** | prendre comme cible la personne la plus proche (la plus grande dans l'image) |
| **Q** ou **Échap** | quitter : le rover s'arrête |

## Suivi de personne
- **Direction** : la personne est gardée au centre de l'image (la caméra est tournée de 90°
  selon le réglage de rotation de la caméra).
- **Distance** visée : 1 m (option `--distance`).
  - Si la personne est droit devant, le **capteur avant** (VL53L1X) mesure la distance.
  - Sinon, le programme se fie à la **taille de la personne dans l'image**.
- Si la personne est perdue plus d'une seconde, le rover **s'arrête et attend**. Il ne
  tourne pas tout seul pour la chercher.
- Réglages (options de `rover_ia.py`) :
  - `--vitesse-max` (110), `--rotation-max` (90) ;
  - `--hauteur-cible` (0,6 = part de l'image occupée par la personne à la bonne distance) ;
  - `--taille` (416 ; plus petit = plus rapide, moins précis).

## Sécurité : les réflexes du rover passent avant l'IA
- Le rover **n'avance pas** si un obstacle est à moins de 25 cm devant, et **ne recule pas**
  si un obstacle est à moins de 15 cm derrière, quoi que demande l'IA.
- Si l'IA se tait 0,4 s (programme planté, WiFi coupé), le rover s'arrête.
- Fermer l'IA, appuyer sur Espace ou utiliser le STOP de la page arrête tout.
- Une commande manuelle (page, joystick, RemoteXY) reprend la main.

## À savoir
- **La vidéo de la carte caméra ne sert qu'un écran à la fois.** Pendant que l'IA tourne, la
  page web affiche « Vidéo masquée » ou n'a pas d'image. Décocher « Afficher la vidéo »
  dans la page laisse le flux à l'IA.
- L'environnement Python (≈ 1 Go) et le modèle sont dans `%LOCALAPPDATA%\RoverIA`, **hors
  du dossier OneDrive** : ils ne sont ni synchronisés ni envoyés sur GitHub.
- La vitesse dépend du WiFi de la caméra : 5 à 8 images/s aujourd'hui (signal -60 à
  -75 dBm). Un meilleur signal rend le suivi plus réactif.
