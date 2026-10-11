#pragma once
// MODELE : copier ce fichier en include/secrets.h et le completer.
// include/secrets.h n'est pas versionne (.gitignore).

#define WIFI_SSID         "nom-du-reseau"
#define WIFI_MOT_DE_PASSE "mot-de-passe-wifi"

// Mot de passe des televersements par WiFi (OTA). Le meme doit
// figurer dans secrets.ini a la racine du projet :
//   [env:test_capteurs_ota]
//   upload_flags = --auth=mot-de-passe-ota
#define OTA_MOT_DE_PASSE  "mot-de-passe-ota"
