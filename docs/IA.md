# IA du rover : YOLO26n sur le PC

```
 Carte caméra ──WiFi, vidéo MJPEG (port 82)──► PC : ia/rover_ia.py (YOLO26n + suivi ByteTrack)
                                                │
 Rover ◄──────────── WiFi, WebSocket ───────────┘
   │  ◄── [IA]{détections}  (5/s)        → relayées aux pages : cadres sur la vidéo
   │  ◄── I vitesse rotation (≤ 10/s)    → seulement pendant un suivi
   │  ──► [DATA] état, [CAM] adresse de la caméra
   │  ──► [POURIA] réglages venus de la page ou de la mission « Suivre »
 Page web ── [POURIA] conf 0.5 | classes person,cup | affichage 1 | suivre person | stop
```

## Lancer

Dans VS Code, menu **Terminal > Exécuter la tâche…** :

| Tâche | Effet |
|---|---|
| **IA rover : voir** | fenêtre vidéo avec les détections, et cadres envoyés à la page |
| **IA rover : service sans fenêtre** | même chose sans fenêtre : tout se règle depuis la page |
| **IA rover : suivre une personne** | fenêtre ; suit une personne après appui sur **G** (démarre en pause) |
| **IA rover : installer / mettre à jour** | (ré)installe Python 3.12, et installe ou met à jour PyTorch, YOLO, OpenCV |

Au premier lancement, le modèle `yolo26n.pt` (environ 6 Mo) se télécharge tout seul.
L'IA se connecte au rover (`rover.local`), annonce sa présence, puis récupère l'adresse
de la caméra. Elle se reconnecte toute seule si le WiFi coupe.

**L'IA ne commande les moteurs que pendant un suivi** : mission « Suivre » lancée depuis
la page (ou `M suivre <objet>` dans la console), ou touche G dans sa fenêtre. Le reste du
temps, elle ne fait que regarder et envoyer ses détections.

Touches, dans la fenêtre vidéo :

| Touche | Effet |
|---|---|
| **G** | démarrer ou reprendre le suivi (personne, ou l'objet demandé par la page) |
| **Espace** | pause : le rover s'arrête |
| **C** | prendre comme cible le plus grand objet de la sorte suivie |
| **Q** ou **Échap** | quitter : le rover s'arrête |

## Mode d'emploi : régler YOLO depuis la page
Section **IA / YOLO** de la page de commande (http://rover.local/) :

| Réglage | Effet | Commande envoyée |
|---|---|---|
| **Confiance** | seuil de 0,05 à 0,95. Plus bas : plus d'objets, plus de fausses détections. 0,45 par défaut ; 0,3 dans une pièce sombre | `[POURIA] conf 0.45` |
| **Objets affichés** | liste des sortes à garder (vide = toutes). Noms anglais de YOLO : `person`, `chair`, `cup`, `bottle`, `dog`, `cat`, `backpack`, `tv`… | `[POURIA] classes person,cup` |
| **Cadres sur la vidéo** | montre ou cache les cadres (l'IA continue de tourner) | `[POURIA] affichage 1` / `0` |
| **Mission « Suivre »** | bouton Suivre personne, ou Suivre objet + sorte choisie | `M suivre person` → le rover relaie `[POURIA] suivre person` |
| **STOP** | arrête la mission ; l'IA cesse de piloter | `M stop` → `[POURIA] stop` |

Les réglages restent valables tant que l'IA tourne. Ils reviennent aux valeurs par défaut
quand on la relance (options `--confiance`, etc.).

Chaque trame `[IA]` reçue par la page contient :
`{"det":[[x,y,l,h,"nom",confiance,id],...],"img":[l,h],"ips":..,"conf":..,"classes":[..],
"affichage":..,"suivi":true|false,"classe":"person","cible":id,"texte":"...","v":..,"r":..}`.
Les coordonnées vont de 0 à 1 dans l'image **remise droite** (rotation de la caméra déjà
appliquée) : la page les dessine directement sur la vidéo tournée.

## Suivi d'une personne ou d'un objet
- **Direction** : l'objet est gardé au centre de l'image. La caméra regarde droit devant,
  donc le rover tourne sur lui-même. Avec le futur support 2 axes, la caméra pourra
  suivre seule avant que le rover tourne.
- **Distance** visée : 1 m (option `--distance`).
  - Si l'objet est droit devant, le **capteur avant** (VL53L1X) mesure la distance.
  - Sinon, le programme se fie à la **taille de l'objet dans l'image**
    (`--hauteur-cible` 0,6 = part de l'image occupée à la bonne distance ; à baisser
    pour un petit objet).
- Si la cible est perdue plus d'une seconde, le rover **s'arrête et attend**. Il reprend
  la plus grande de la même sorte dès qu'elle réapparaît. Il ne tourne pas tout seul pour
  la chercher (à améliorer).
- Options de `rover_ia.py` : `--vitesse-max` (110), `--rotation-max` (90), `--taille`
  (416 ; plus petit = plus rapide, moins précis).
- Les ordres partent une fois par image analysée, 10 par seconde au plus : en pratique
  6 à 10 par seconde selon la vidéo. Si la vidéo se fige, les ordres cessent et le rover
  s'arrête au bout de 0,4 s (il ne roule pas à l'aveugle).

## Choisir le modèle
Option `--modele` (par défaut `%LOCALAPPDATA%\RoverIA\yolo26n.pt`). Un modèle absent se
télécharge tout seul au premier lancement. Comparaison faite le 20/10/2026 (ultralytics
8.4.181, PyTorch 2.14 CPU, image 416) : les trois fonctionnent avec ce programme, suivi
ByteTrack compris, à la même vitesse (environ 22 ms par image sur ce PC).

| Modèle | Photo « 2 personnes » (Ultralytics) | Remarque |
|---|---|---|
| `yolo26n.pt` | personnes 0,87 et 0,86 · cravate 0,51 | **par défaut** : confiance plus haute sur les personnes, pas de fausse personne |
| `yolo11n.pt` | personnes 0,86 et 0,85 | ne voit pas les cravates |
| `yolov8n.pt` | personnes 0,91 et 0,71 · une fausse personne à 0,27 | ancien modèle par défaut |

Revenir à l'ancien modèle : `rover_ia.py --modele %LOCALAPPDATA%\RoverIA\yolov8n.pt`.

## Sécurité : les réflexes du rover passent avant l'IA
- Le rover **n'avance pas** si un obstacle est à moins de 25 cm devant, et **ne recule pas**
  si un obstacle est à moins de 15 cm derrière, quoi que demande l'IA.
- Si l'IA se tait 0,4 s (programme planté, WiFi coupé), le rover s'arrête. Si elle se
  déconnecte pendant une mission « Suivre », la mission s'arrête.
- Fermer l'IA, appuyer sur Espace ou sur le STOP de la page arrête tout.
- Une commande manuelle (page, joystick, RemoteXY) reprend la main et arrête la mission.
- Sans IA connectée, le rover refuse la mission « Suivre » avec un message.

## À savoir
- La carte caméra a **deux flux vidéo** : le port 81 pour la page, le port 82 pour l'IA.
  Chacun sert un seul écran à la fois ; si le flux de la page est déjà pris (page ouverte
  sur le PC et sur le téléphone), la page passe toute seule en « image par image ».
  Les flux se partagent le WiFi de la caméra :
  environ 10 images/s pour l'IA seule, et 5 en tout quand la page affiche aussi la vidéo.
- L'environnement Python (≈ 1 Go) et le modèle sont dans `%LOCALAPPDATA%\RoverIA`, **hors
  du dossier OneDrive** : ils ne sont ni synchronisés ni envoyés sur GitHub.
- Un meilleur signal WiFi de la caméra (aujourd'hui -60 dBm) rend le suivi plus réactif.
- Pour faire évoluer l'IA avec Claude : [prompt_claude_yolo.md](prompt_claude_yolo.md).
