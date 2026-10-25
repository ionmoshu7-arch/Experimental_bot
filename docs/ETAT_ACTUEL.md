# État actuel du rover — 25/10/2026

Photographie du projet : ce qui est câblé, les valeurs réellement utilisées par le code,
ce qui est validé, ce qui est prévu et les points ouverts.
Sources : `lib/Rover/src/Config.h` (référence du brochage), le code cité à chaque tableau,
[schema_rover.html](schema_rover.html) et les mesures faites pendant les essais.
Aucun identifiant WiFi ni mot de passe ici : ils sont dans `include/secrets.h` et
`secrets.ini`, hors de git.

Légende : **validé** = essayé sur le matériel avec un résultat mesuré · **à valider** =
codé, pas encore essayé (ou essai sans conclusion).

---

## 1. Matériel

| Élément | Modèle | État |
|---|---|---|
| Carte rover | ESP32-S3 DevKitC-1 (puce USB CH343, n° de série 5C38182766) | validé |
| Carte caméra | ESP32-S3 N16R8 « Diymore ESP32 S3 CAM » + OV2640 + lecteur TF (puce USB CH340, 1A86:7523) | validé |
| Batterie | 18 V, 4 Ah (72 Wh), interrupteur général | validé |
| Alimentations | 3 × XL4005 : 3,3 V (capteurs, encodeurs, logique TB6612) · 5,6 V (ESP32 via 1N4007 → ≈ 4,9 V, servos prévus) · 9 V (VM des TB6612) | validé, réglées hors charge |
| Moteurs | 4 × TT 1:48, 9 V, 0,25 A nominal / 0,6 A bloqué | validé |
| Ponts en H | 2 × TB6612FNG (#1 côté droit, #2 côté gauche), STBY commun | validé |
| Encodeurs | 4 × quadrature, 576 impulsions/tour (3 × 48 × 4), alimentés en 3,3 V | validé |
| Caméra montée sur le rover | broche 5V de la carte, depuis le rail 5,6 V via 1N4007 | montée (alimentation à surveiller, voir § 8) |

---

## 2. Brochage du rover (ESP32-S3 DevKitC-1)

Source : `lib/Rover/src/Config.h`. Les inversions de câblage sont corrigées **dans le
logiciel** (le câblage physique suit le PDF `Rover_Cablage_Moteurs.pdf`).

### Moteurs (TB6612FNG)

| Moteur | Position | PWM | IN1 | IN2 | Remarque |
|---|---|---|---|---|---|
| M1 | avant-droit | GPIO2 | GPIO4 | GPIO1 | IN1/IN2 échangées en logiciel (moteur à l'envers) |
| M2 | avant-gauche | GPIO10 | GPIO11 | GPIO12 | |
| M3 | arrière-droit | GPIO5 | GPIO6 | GPIO7 | |
| M4 | arrière-gauche | GPIO13 | GPIO15 | GPIO14 | IN1/IN2 échangées en logiciel (moteur à l'envers) |
| STBY (les 2 modules) | | GPIO16 | | | sécurité : coupe tous les moteurs |

### Encodeurs

| Moteur | Phase A | Phase B | Remarque |
|---|---|---|---|
| M1 | GPIO40 | GPIO41 | |
| M2 | GPIO19 | GPIO18 | A/B échangées en logiciel (positif = avance) |
| M3 | GPIO47 | GPIO48 | GPIO48 = LED RGB de la carte v1.0 : LED inutilisable |
| M4 | GPIO17 | GPIO21 | A/B échangées en logiciel (positif = avance) |

### Bus et autres broches

| Fonction | GPIO | Remarque |
|---|---|---|
| I2C SDA | GPIO38 | 100 kHz |
| I2C SCL | GPIO39 | |
| Port série (moniteur) | GPIO43 TX / GPIO44 RX | à garder libre |
| Libres, avec précautions | GPIO0, 3, 45, 46 (démarrage) · 35–37 (PSRAM sur modules R8) · GPIO20 (USB D+) | non utilisés |
| Prévus (non câblés) | GPIO42 buzzer · GPIO8 micro analogique · GPIO9 photorésistance (+ 10 kΩ) | voir § 7 |

---

## 3. Adresses I2C

Bus principal, 100 kHz (`Config.h`, `Inventaire.cpp`).

| Adresse | Composant | État |
|---|---|---|
| 0x68 | MPU-6500 gyroscope + accéléromètre (AD0 à la masse, WHO_AM_I = 0x70) | **câblé, validé** |
| 0x73 | TCA9548A multiplexeur (A0/A1 soudés sur VCC) | **câblé, validé** |
| 0x29 | (derrière le multiplexeur) VL53L1X, 3 × VL53L0X, 3 × TCS34725 | **câblés, validés** |
| 0x40 | PCA9685 #1 : servos (radar, support caméra 2 axes) | prévu |
| 0x41 | PCA9685 #2 : LED RGB | prévu |
| 0x20 | MCP23017 : 4 LED + 3 boutons | prévu |
| 0x38 | DHT20 température / humidité | prévu |
| 0x3C | écran OLED | prévu |
| 0x48 | ADS1115 convertisseur 4 voies | prévu |
| 0x50 | ESP32-S3-CAM en esclave I2C | prévu (aujourd'hui la caméra parle au rover par WiFi) |

### Canaux du TCA9548A

| Canal | Capteur | Portée réelle utilisée | État |
|---|---|---|---|
| 0 | VL53L1X avant | 4000 mm (mode Long) | validé |
| 1 | VL53L0X arrière | 1200 mm | validé |
| 2 | VL53L0X droite | **250 mm seulement** (au-delà : code 8190/8191) | fonctionne, portée réduite |
| 3 | VL53L0X gauche | 600 mm | validé, à surveiller |
| 4 | TCS34725 sol gauche | — | validé |
| 5 | TCS34725 sol centre | — | validé |
| 6 | TCS34725 sol droit | — | validé |
| 7 | libre | (+ un 2e TCA9548A en réserve) | — |

---

## 4. Carte caméra (ESP32-S3 N16R8)

| Fonction | Valeur | Source |
|---|---|---|
| Brochage caméra | profil « Freenove / ESP32-S3-EYE » détecté automatiquement : PWDN −1, RESET −1, XCLK 15, SIOD 4, SIOC 5, Y2–Y9 = 11, 9, 8, 10, 12, 18, 17, 16, VSYNC 6, HREF 7, PCLK 13 | `src/camera/Camera.cpp` |
| Carte TF (SD_MMC 1 bit) | CLK 39, CMD 38, D0 40 | `Stockage.cpp` |
| Horloge caméra | XCLK 10 MHz | `Camera.cpp` |
| Image | VGA 640 × 480, qualité JPEG 12, 2 tampons en PSRAM, rotation 90° (gardée en mémoire) | `Camera.cpp` |
| Port série | 460800 bauds (CH340 instable à 921600) | `Pont.h` |
| Puissance WiFi | **15 dBm** (voir § 6) | `src/camera/main.cpp` |

---

## 5. Calibrations et paramètres

Les valeurs marquées « mémoire » sont gardées dans la mémoire du rover (NVS) et peuvent
avoir été changées depuis la page : la valeur active s'y affiche.

### Odométrie, gyroscope, terrain

| Paramètre | Valeur | Source |
|---|---|---|
| Impulsions par tour de roue | 576 | `Config.h` |
| Diamètre de roue | 6,7 cm → 0,365 mm par impulsion | `Config.h` |
| Gyroscope | ±250 °/s ; biais de l'axe Z mesuré à chaque démarrage (1500 mesures, rover immobile) ; recalibrage : commande `c` | `Gyro.cpp` |
| Pente | zéro pris au démarrage (sol plat supposé) ; commande `T` pour refaire | `Terrain.cpp` |
| Bosses | seuil 0,3 g, 300 ms entre deux bosses | `Terrain.cpp` |
| Glissement | mesuré au-dessus de 50 mm/s, sur 250 ms, obstacle devant à moins de 2 m | `Terrain.cpp` |

### Capteurs de distance et de couleur

| Paramètre | Valeur | Source |
|---|---|---|
| VL53L1X avant | mode Long, budget 33 ms (≈ 30 mesures/s) | `Distance.cpp` |
| VL53L0X | mesure continue, délai max 200 ms ; code d'erreur 8190 = hors portée | `Distance.cpp`, `Distance.h` |
| Mesure trop ancienne | > 500 ms → capteur « sans mesure » | `Distance.h` |
| TCS34725 | intégration 24 ms (ATIME 0xF6, ≈ 40 mesures/s), gain 16× | `Couleur.cpp` |
| Classement des couleurs | noir < 0,03 de luminosité relative ; blanc > 0,30 sans dominante ; dominante > 0,42 | `Couleur.cpp` |
| Capteurs de couleur on/off | commande `U 0/1`, mémoire | `Couleur.cpp` |

### Suivi de ligne

| Paramètre | Valeur | Source |
|---|---|---|
| Kp / Kd / vitesse | **80 / 4 / 90** par défaut, mémoire (commande `P kp kd v`) | `SuiviLigne.cpp` |
| Calibration | blanc et noir par capteur, sur le canal clair, mémoire (bouton Calibrer, 4 s) ; contraste minimal 1,3 | `SuiviLigne.cpp` |
| Ligne perdue | somme des noirceurs < 0,35 ; tolérance 150 ms en ligne droite | `SuiviLigne.cpp` |
| Recherche | activée, stratégie « toutes » ; 12 s au plus, 4 s par étape | `SuiviLigne.cpp` |
| Régulation | toutes les 25 ms | `SuiviLigne.cpp` |
| Obstacle | pause sous 150 mm, reprise au-delà de 220 mm | `SuiviLigne.cpp` |

### Navigation autonome

| Paramètre | Valeur | Source |
|---|---|---|
| Vitesse de croisière | 110 / 255 (réglable 60–200, commande `Q`) | `Navigation.cpp` |
| Tenue de cap (PD) | Kp 4,0 par degré · Kd 0,4 par °/s | `Navigation.cpp` |
| Virage au gyroscope | Kp 2,5 ; vitesse 75–140 ; tolérance 3° ; 5 s au plus | `Navigation.cpp` |
| Obstacle devant | ralentit dès 600 mm (jusqu'à 55 %), s'arrête à 280 mm, recule sous 160 mm | `Navigation.cpp` |
| Recul | 12 cm (grand recul 20 cm), vitesse 90 ; place derrière ≥ 280 mm, arrêt à 150 mm ; 3 s au plus | `Navigation.cpp` |
| Écart des murs | gauche 150 mm, droite 120 mm, correction 25 °/s | `Navigation.cpp` |
| Collision | choc > 0,35 g · roue < 35 % du rendement appris pendant 300 ms (consigne ≥ 50) · rover poussé > 20° de son cap · virage bloqué (< 8 °/s pendant 700 ms) ; 500 ms ignorées au départ d'une manœuvre | `Navigation.cpp` |
| Collisions répétées | 3 en 15 s : grand recul et demi-tour · 6 en 30 s : dégagement | `Navigation.cpp` |
| Dégagement | tour complet en 24 secteurs de 15°, vitesse 85, 12 s au plus ; sortie si ≥ 350 mm libres ; secousses à 200 (phases de 350 ms) ; 4 essais | `Navigation.cpp` |
| Options | collision, dégagement, tenue de cap, écart des murs : toutes **activées** par défaut, mémoire (commande `Y`) | `Navigation.cpp` |

### Missions

| Paramètre | Valeur | Source |
|---|---|---|
| Arrivée à un but | à moins de 150 mm ; virage sur place d'abord si l'écart de cap dépasse 60° | `Navigation.cpp` |
| Longer un mur | gauche 150 mm, droite 120 mm ; Kp 0,3 °/s par mm ; recherche du mur à 20 °/s | `Navigation.cpp` |
| Patrouille | 8 points au plus ; pause 1,5 s à chaque point ; photo optionnelle | `Missions.cpp`, `Missions.h` |
| Exploration | 5 min par défaut | `Missions.cpp` |

### Sécurité (toujours active)

| Paramètre | Valeur | Source |
|---|---|---|
| Plus de commande moteur | arrêt après 400 ms | `Commandes.h` |
| Ordres de l'IA | pas d'avance si obstacle < 250 mm devant, pas de recul si < 150 mm derrière | `Commandes.cpp` |
| Page fermée / WiFi perdu / mise à jour | arrêt général | `main.cpp` |
| IA déconnectée | arrêt si elle pilotait | `main.cpp` |

### IA du PC (YOLO)

| Paramètre | Valeur | Source |
|---|---|---|
| Modèle | **yolo26n** (yolov8n et yolo11n essayés, même vitesse) + suivi ByteTrack | `ia/rover_ia.py` |
| Confiance / taille d'image | 0,45 / 416 | `ia/rover_ia.py` |
| Suivi | distance 1,0 m ; hauteur cible 0,6 de l'image ; vitesse max 110 ; rotation max 90 | `ia/rover_ia.py` |
| Cadences | ordres ≤ 10/s, détections vers la page 5/s | `ia/rover_ia.py` |
| Versions | Python 3.12, Ultralytics 8.4.181, PyTorch 2.14.1 (CPU), OpenCV 5.0.0 ; environnement `%LOCALAPPDATA%\RoverIA\venv` | mesuré le 20/10 |

---

## 6. Fréquences et cadences

| Élément | Valeur | Source |
|---|---|---|
| PWM moteurs | 20 kHz, 8 bits | `Config.h` |
| Bus I2C | 100 kHz | `Config.h` |
| Port série du rover | 921600 bauds | `main.cpp` |
| Trames `[DATA]` (pages, IA) | 20/s (toutes les 50 ms) ; texte lisible 1/s | `Telemetrie.h` |
| Trames vers la caméra (journal) | 5/s (10/s pendant un enregistrement) | `Reseau.cpp` |
| Boucle principale | ≈ 38 Hz observé avec le WiFi actif (page du 18/10) | mesuré |
| WebSocket | ping toutes les 2 s ; client muet déconnecté après ≈ 8 s ; 8 clients au plus | `Reseau.cpp`, `platformio.ini` |
| Journal (carte TF) | nouveau fichier toutes les 10 min ; 288 fichiers (48 h) ou 2 Go au plus ; écriture toutes les 2 s | `Journal.h`, `Journal.cpp` |
| Vidéo enregistrée | 10 images/s au plus, 10 min par fichier, arrêt si moins de 20 Mo libres | `Enregistrement.h/.cpp` |
| Puissance WiFi | rover 19,5 dBm (max) · caméra 15 dBm | `Reseau.cpp`, `camera/main.cpp` |

Essai A/B du 20/10 (3 tours, flux vidéo de l'IA) : caméra à 19,5 dBm → 1,7 images/s ; à
15 dBm → 5,0 ; à 11 dBm → 5,9. La pleine puissance dégrade la vidéo de cette carte.

### Réseau

| Carte | Adresse | Ports |
|---|---|---|
| Rover | `rover.local` | 80 page web · 81 WebSocket · 6377 RemoteXY · `POST /maj` (mot de passe) |
| Caméra | `camera.local` | 80 HTTP (état, réglages, photos, carte TF, `/maj`) · 81 vidéo de la page · 82 vidéo de l'IA |

Protections d'accès : voir [Architecture.md](Architecture.md), « Sécurité et données
personnelles ».

---

## 7. Ce qui est validé, à valider, prévu

### Validé sur le matériel

- Tous les capteurs I2C répondent quand la batterie est allumée (inventaire au démarrage).
- Télémétrie 20/s, page web servie par le rover, connexion USB et WiFi.
- Mise à jour du rover par le câble de la caméra (57 s, le 18/10) et par WiFi direct
  (≈ 45 s) ; mise à jour de la caméra par câble et par WiFi.
- Caméra : 2 flux vidéo simultanés (page + IA), mode « image par image » si le flux est
  pris, photos, enregistrement, carte TF vidée à distance (le 18/10).
- Journal de navigation : fichiers créés et renouvelés toutes les 10 min (le 20/10, 30
  fichiers, 57,6 Mo).
- Rover à l'arrêt : capteurs de couleur on/off (`U`), options (`Y`) gardées après
  redémarrage, points de patrouille (`M point`, `M effacer`), refus de « suivre » sans IA.
- IA : détections dans la page, réglages depuis la page (confiance, objets, cadres),
  mission « Suivre » démarrée et arrêtée (essai avec moteurs bridés à 0).
- Sécurité : connexions WebSocket d'origine inconnue refusées ; caméra : accès d'un autre
  site, « DNS rebinding » et actions sans en-tête `X-Rover` refusés (le 20/10).
- WiFi : 8 clients, déconnexion des clients muets (le 20/10).

### À valider (codé, pas encore essayé ou sans conclusion)

- Missions qui roulent : Explorer, Patrouille, Mur gauche/droit (essais par l'utilisateur,
  rover au sol).
- Retour base : lancé le 20/10, le rover était déjà à la base (« but atteint » aussitôt).
- Suivre une personne en roulant : essayé le 20/10, la personne est détectée mais la cible
  change souvent (identifiant #45 → #82) : suivi saccadé.
- Suivi de ligne en roulant : calibré, réglages par défaut ; performance à confirmer.
- Effacement automatique du journal après 48 h / 2 Go (pas encore atteint).

### Prévu

- Support caméra 2 axes (pan/tilt) : servos sur le PCA9685 #1 (0x40), canaux 1 et 2 ;
  constantes déjà dans `Config.h`, pas encore codé ; bloc désactivé dans la page.
- Servo radar (PCA9685 #1, canal 0), LED RGB (PCA9685 #2), MCP23017 (4 LED + 3 boutons),
  OLED, DHT20, ADS1115, buzzer (GPIO42), micro analogique (GPIO8), photorésistance (GPIO9).
- Achats envisagés : micro I2S INMP441, ampli I2S MAX98357A + haut-parleur, VL53L1X pour
  l'arrière.
- Mot de passe sur la page web et sur RemoteXY (reporté).
- Architecture « à brancher » (modules enregistrés, une mission par fichier) : idée
  discutée, pas décidée.

---

## 8. Points ouverts

| Point | Détail | Piste |
|---|---|---|
| VL53L0X droite | 250 mm de portée seulement | remplacer, ou VL53L1X |
| VL53L0X gauche | 600 mm, à surveiller | |
| Position des capteurs de distance sur la carte | montage estimé dans la page (avant/arrière à 120 mm du centre, côtés à 80 mm) | mesurer sur le rover |
| Alimentation | rover sorti du WiFi le 18/10 (≈ 5 min) ; capteurs absents une fois (batterie coupée ?) ; la caméra se dégrade à pleine puissance WiFi | vérifier batterie, rail 5,6 V, diode 1N4007 |
| Débit vidéo | ≈ 4–6 images/s au total, partagé entre la page et l'IA (signal −62 à −67 dBm) | antenne externe sur la caméra |
| Suivi de personne | cible perdue et reprise souvent (≈ 3 images/s analysées) | garder la boîte la plus proche de la précédente |
| IA arrêtée seule | code de sortie 4 le 20/10, cause inconnue | relancer et lire le journal |
| Historique GitHub | nom du réseau WiFi et e-mail personnel encore dans les anciens commits | réécriture préparée (sauvegarde faite), en attente d'autorisation |
| LED RGB de la carte | GPIO48 = encodeur M3 B : LED inutilisable | |
| Compilation dans OneDrive | une fois 17 min (habituellement 20 s à 2 min) | exclure `.pio` de la synchronisation si ça revient |
