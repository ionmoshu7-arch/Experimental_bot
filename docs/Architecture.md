# Architecture du rover : 2 cartes ESP32-S3 + PC

Schémas : [architecture de fonctionnement](architecture_rover.html) ([PNG](architecture_rover.png))
· [schéma électrique](schema_rover.html) ([PNG](schema_rover.png))
· page de commande : http://rover.local/ · caméra : http://camera.local/

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
| `M explorer 5` | mission : explorer 5 min (navigation libre, la carte se remplit) |
| `M base` | mission : retour au point (0, 0), puis face au cap de départ |
| `M point` / `M effacer` | mémoriser la position actuelle comme point de patrouille (8 au plus) / tout effacer |
| `M patrouille` | parcourir les points en boucle (pause 1,5 s à chaque point) |
| `M photos 1` / `0` | photo par la caméra à chaque point de patrouille |
| `M mur g` / `M mur d` | longer le mur gauche (15 cm) / droit (12 cm) |
| `M suivre person` | suivre une personne (ou un autre objet YOLO) avec l'IA du PC |
| `M stop` | arrêter la mission |
| `Y collision 0` / `1` | option : détection de collision (aussi `degagement`, `cap`, `murs`) ; gardée après redémarrage |
| `U 0` / `U 1` | capteurs de couleur éteints / allumés (gardé après redémarrage) |

- **Commandes pour la caméra** (préfixe `cam`) :

| Commande | Effet |
|---|---|
| `cam photo` | photo sur la carte TF |
| `cam video 1` / `cam video 0` | lancer / arrêter l'enregistrement |
| `cam res VGA` | résolution (QVGA, VGA, SVGA, XGA, HD, SXGA, UXGA) |
| `cam etat` | état complet, dont la version |
| `cam journal` | fichiers du journal de navigation |
| `cam vider oui` | effacer toute la carte TF (photos, vidéos, journal) |
| `cam liaison 0` / `1` | couper / rétablir la liaison avec le rover |
| `cam` | aide |

## Alimentation de la caméra montée sur le rover

Broche **5V** de la carte caméra, depuis le rail 5,6 V, à travers une diode 1N4007 (≈ 4,9 V).
Pas le rail 3,3 V des capteurs : les pointes de courant du WiFi le perturberaient.

## Journal de navigation

La caméra écrit sur sa carte TF tout ce que le rover envoie : ses mesures `[DATA]`
(5 fois par seconde, 10 pendant un enregistrement vidéo) et ses messages (`[NAV]`,
`[MISSION]`, `[LIGNE]`...).
- Fichiers `/journal/J<démarrage>_<n>.jsonl`, une ligne = `millisecondes<tab>message`.
- Nouveau fichier toutes les 10 minutes ; les plus anciens sont effacés au-delà de 288
  fichiers (48 h) ou 2 Go.
- Liste et téléchargement : section « Journal de navigation » de la page, ou
  http://camera.local/journaux.

## IA (YOLO26n sur le PC)

Voir [IA.md](IA.md) : détections dans la page, réglages depuis la page, mission « Suivre ».
Pour la faire évoluer avec Claude : [prompt_claude_yolo.md](prompt_claude_yolo.md).
Les réflexes de sécurité du rover restent prioritaires.

## Sécurité et données personnelles

La caméra filme l'intérieur de la maison : vidéo, photos et journal sont des données
personnelles. Protections en place (20/10/2026) :

| Zone | Protection |
|---|---|
| WebSocket du rover (:81) | seules la page du rover (`rover.local` ou son IP), l'IA du PC et la caméra peuvent se connecter : un site Internet ouvert dans un navigateur du réseau ne peut plus piloter le rover ni lire ses données (contrôle de l'en-tête `Origin`) |
| Caméra (:80, :81, :82) | la requête doit viser `camera.local` ou l'IP de la caméra (bloque le « DNS rebinding ») ; seules les pages du rover et de la caméra peuvent lire les réponses (fin du `Access-Control-Allow-Origin: *`) |
| Actions de la caméra (photo, réglages, vider la carte) | en-tête `X-Rover` obligatoire : un lien ou une image piégée sur un autre site ne peut plus les déclencher |
| Mises à jour (`/maj`, OTA) | mot de passe de `include/secrets.h` / `secrets.ini` |
| Dépôt GitHub (public) | mots de passe et nom du réseau WiFi jamais publiés (vérifié dans tout l'historique) ; photos, vidéos et journaux exclus par `.gitignore` ; l'IA n'enregistre aucune image |

Conséquences pratiques :
- Ouvrir la page par **http://rover.local/** (ou l'IP du rover). Ouverte comme fichier
  local, elle ne peut plus se connecter en WiFi (le câble USB marche toujours).
- Un outil maison qui appelle la caméra doit viser `camera.local` ou son IP, et ajouter
  l'en-tête `X-Rover: 1` pour les actions.

Limites connues : sur le réseau de la maison, quelqu'un qui ouvre la page du rover peut
toujours piloter le rover et voir la vidéo (pas de mot de passe sur la page ni sur
RemoteXY), et les échanges ne sont pas chiffrés (HTTP). Ne pas exposer le rover ni la
caméra sur Internet (pas de redirection de port sur la box), et garder un mot de passe
WiFi solide.
