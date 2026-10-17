# Piloter le rover avec RemoteXY (téléphone Android)

Le firmware `test_capteurs` contient déjà tout le code RemoteXY
([RemoteXYControle.cpp](../src/test_capteurs/RemoteXYControle.cpp)). Il ne manque que
**l'interface** (l'écran du téléphone), qui se dessine sur le site RemoteXY.

Le rover reste sur le WiFi « Voyager 1 » : la page web, le WebSocket et les mises à jour
par WiFi continuent de fonctionner en même temps.

## 1. Créer l'interface sur remotexy.com (sur le PC)

1. Ouvrir **https://remotexy.com/en/editor/**.
2. Panneau **Configuration** (à droite) :
   - Connexion : **WiFi** en mode **client** (le rover se connecte au réseau de la maison) ;
   - Carte : **ESP32** ; module : **WiFi intégré à l'ESP32** ; environnement : **Arduino IDE** ;
   - Port : **6377**. Le nom et le mot de passe WiFi demandés ici sont ignorés par le firmware.
3. Poser les éléments ci-dessous et, pour chacun, renseigner le **nom de variable**
   (champ « Variable name » dans les propriétés de l'élément), **exactement** comme indiqué :

| Élément RemoteXY | Nom de variable | Réglage | Rôle sur le rover |
|---|---|---|---|
| Joystick | `joystick` | revient au centre | pilotage manuel (relâché = arrêt) |
| Slider | `slider_vitesse` | 0 à 100 | vitesse maximale du joystick, en % |
| Select (3 positions) | `selecteur_mode` | A, B, C | A = manuel, B = suivi de ligne, C = navigation |
| Button | `button_stop` | rouge | STOP : arrête tout, sélecteur sur A |

   C'est l'interface en place dans
   [RemoteXYInterface.h](../src/test_capteurs/RemoteXYInterface.h) (version gratuite).
   Elle n'affiche pas d'état sur le téléphone : la page web reste là pour ça.

   Ajouter des libellés (« Ligne », « Navigation », « STOP »...) avec l'élément *Label* si
   on le souhaite : ils n'ont pas de variable et ne changent rien au code.

4. Pour modifier l'écran : cliquer sur **Get source code** et copier le bloc qui va de
   `#pragma pack(push, 1)` à `#pragma pack(pop)` (le tableau `RemoteXY_CONF_PROGMEM`
   et la `struct ... RemoteXY;`).
5. Le coller dans un nouveau fichier **`src/test_capteurs/RemoteXYInterface.h`**
   (ou le coller dans la discussion avec Claude, qui l'intègre).
6. Téléverser `test_capteurs`. Au démarrage, le moniteur série affiche
   `RemoteXY  port 6377`.

Si la compilation signale un nom inconnu (par exemple `joystick_x`), un nom de variable
ne correspond pas au tableau ci-dessus : le corriger dans l'éditeur et recopier le code.

## 2. Connecter le téléphone Samsung

1. Installer **RemoteXY** depuis le Google Play Store.
2. Mettre le téléphone sur le WiFi **Voyager 1**.
3. Dans l'application : **+** → **WiFi** (connexion TCP) →
   adresse **192.168.129.13**, port **6377** → connexion.
   L'adresse s'affiche aussi au démarrage du rover (`[WIFI] connecte : ...`)
   et dans l'en-tête de la page web.
4. L'interface dessinée apparaît : le rover se pilote au joystick.

Astuce : réserver l'adresse 192.168.129.13 au rover dans la box (bail DHCP fixe)
évite qu'elle change un jour.

## Sécurité

- Joystick relâché : les moteurs s'arrêtent.
- Application fermée ou WiFi perdu : tout s'arrête (moteurs, suivi de ligne, navigation).
- Toucher le joystick reprend la main sur le suivi de ligne ou la navigation
  (le sélecteur revient sur A).
- Un mode qui s'arrête tout seul (ligne perdue, pas de calibration...) remet
  aussi le sélecteur sur A.
- Le bouton STOP de la page web et la touche Espace restent actifs en parallèle.
