# Architecture du rover : 2 cartes ESP32-S3 + PC

```
 PC (VS Code) ──câble USB 460800──► ESP32-CAM ◄──WiFi WebSocket──► ESP32 ROVER
   │   console des 2 cartes          │  relaie : programme du rover,     ▲
   │   programmes des 2 cartes       │  commandes, messages              │
   └──── WiFi : vidéo, page web, mises à jour directes ────────────────────┘
 Téléphone ── WiFi ──► page web / RemoteXY du rover
```

| Carte | Rôle | Programme |
|---|---|---|
| **Rover** (ESP32-S3, puce USB CH343) | moteurs, capteurs, sécurité, suivi de ligne, navigation, carte et terrain, page web, WebSocket, RemoteXY | [src/test_capteurs](../src/test_capteurs) |
| **Caméra** (ESP32-S3 N16R8 + OV2640, puce USB CH340) | vidéo, photos, vidéo + mesures, console des 2 cartes, relais de programmation du rover | [src/camera](../src/camera) |

## Programmer les cartes

Choisir l'environnement dans la barre d'état de VS Code, puis cliquer sur → (Upload).

| Environnement | Carte | Chemin |
|---|---|---|
| `camera` | caméra | câble USB de la caméra |
| `test_capteurs_via_camera` | rover | câble de la caméra → WiFi → rover (**seul le câble de la caméra est nécessaire**) |
| `test_capteurs_ota` | rover | WiFi, du PC directement au rover |
| `camera_ota` | caméra | WiFi, du PC directement à la caméra (quand elle est montée sur le rover) |
| `test_capteurs` | rover | câble USB du rover |

- **Avant `test_capteurs_via_camera` : fermer le moniteur série de la caméra**, car le programme passe par ce même port.
- Les envois par WiFi sont faits par le PC en HTTP (`POST /maj`), protégés par le mot de
  passe OTA (`include/secrets.h` et `secrets.ini`, jamais envoyés sur GitHub). Le Pare-feu
  Windows ne les bloque pas.
- Pendant une mise à jour, les moteurs du rover s'arrêtent et l'enregistrement vidéo se coupe.
- Chaque carte affiche sa version (date et heure de compilation) : au démarrage, dans
  `cam etat` et dans les données du rover.
- Mesuré : rover via la caméra en environ 30 s (22 s de câble, puis 7 s de WiFi) ; caméra
  par WiFi en 3 s.

## Console des deux cartes

Ouvrir le moniteur série de VS Code sur l'environnement `camera` (460800 bauds).

- Les messages du rover s'affichent avec le préfixe `[ROVER]` ; son état est résumé toutes les 2 s.
- **Commandes envoyées au rover** (taper puis Entrée) :

| Commande | Effet |
|---|---|
| `s` | arrêt de tout |
| `c` | calibrer le gyroscope (rover immobile) |
| `z` | cap à zéro |
| `O` | nouvelle origine de la carte |
| `T` | pente = 0 (sol plat) |
| `L 1` / `L 0` | suivi de ligne marche / arrêt |
| `N 1` / `N 0` | navigation autonome marche / arrêt |
| `K` | calibrer le suivi de ligne (4 s) |

- **Commandes pour la caméra** (préfixe `cam`) :

| Commande | Effet |
|---|---|
| `cam photo` | photo sur la carte TF |
| `cam video 1` / `cam video 0` | lancer / arrêter l'enregistrement |
| `cam res VGA` | résolution (QVGA, VGA, SVGA, XGA, HD, SXGA, UXGA) |
| `cam etat` | état complet, dont la version |
| `cam liaison 0` / `1` | couper / rétablir la liaison avec le rover |
| `cam` | aide |

## Alimentation de la caméra montée sur le rover

Broche **5V** de la carte caméra, depuis le rail 5,6 V, à travers une diode 1N4007 (≈ 4,9 V).
Pas le rail 3,3 V des capteurs : les pointes de courant du WiFi le perturberaient.

## Prochaine étape : IA

YOLOv8n en Python sur le PC. Elle lit la vidéo de la caméra par WiFi et envoie ses ordres
au rover par le WebSocket, pour le suivi de personne et les obstacles vus par la caméra.
Les réflexes de sécurité du rover restent prioritaires.
