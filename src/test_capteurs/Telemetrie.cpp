#include "Telemetrie.h"

#include <Arduino.h>
#include "Config.h"
#include "Couleur.h"
#include "Distance.h"
#include "Gyro.h"
#include "Moteurs.h"
#include "Reseau.h"

static unsigned long dernierTexte = 0, dernierJson = 0;
static unsigned long nbBoucles = 0, dernierCalculHz = 0;
static unsigned long boucleHz = 0;

static void afficherTexte() {
  char buf[40];
  Serial.println();
  Serial.printf("-------- t=%.1f s   boucle %lu Hz --------\n",
                millis() / 1000.0, boucleHz);

  Serial.print("DISTANCE ");
  for (uint8_t i = 0; i < 4; i++) {
    distanceTexte(i, buf, sizeof(buf), true);
    Serial.printf("  %s %-10s", tof[i].nom, buf);
  }
  Serial.println();

  for (auto &k : couleurs) {
    Serial.printf("COULEUR   %-10s ", k.nom);
    if (!k.present)      Serial.println("ABSENT");
    else if (!k.valide)  Serial.println("pas encore de mesure");
    else Serial.printf("R=%5u G=%5u B=%5u C=%5u  lum=%.2f  %5.0f lux  %5d K  -> %s\n",
                       k.r, k.g, k.b, k.c, k.lum, k.lux, k.cct, k.couleur);
  }

  if (gyroPresent()) {
    const GyroData &g = gyroBrut();
    const AccelData &a = gyroAccel();
    Serial.printf("GYRO      cap=%7.1f deg   gyro x=%6.2f y=%6.2f z=%6.2f deg/s\n",
                  gyroCap(), g.gyroX, g.gyroY, gyroVitesseZ());
    Serial.printf("ACCEL     x=%5.2f y=%5.2f z=%5.2f g   temp=%.1f C\n",
                  a.accelX, a.accelY, a.accelZ, gyroTemperature());
  } else {
    Serial.println("GYRO      ABSENT");
  }

  Serial.print("MOTEURS  ");
  for (int i = 0; i < 4; i++) {
    const Moteur &m = moteurs[i];
    Serial.printf("  %.2s cmd=%4d v=%5.0f mm/s enc=%ld",
                  m.nom, m.consigne, m.vitesse, moteurCompte(i));
  }
  Serial.println();

  if (reseauConnecte())
    Serial.printf("WIFI      http://%s.local/  %s  %d dBm  %d client(s)\n",
                  NOM_HOTE, reseauIp().c_str(), reseauRssi(), reseauNbClients());
  else
    Serial.println("WIFI      non connecte");
}

// La trame est construite une fois puis envoyee au port serie et au WiFi
static char trame[2048];
static size_t lgTrame = 0;

static void ajouter(const char* format, ...) {
  va_list args;
  va_start(args, format);
  int n = vsnprintf(trame + lgTrame, sizeof(trame) - lgTrame, format, args);
  va_end(args);
  if (n > 0) lgTrame = min(lgTrame + n, sizeof(trame) - 1);
}

void journal(const char* format, ...) {
  char ligne[160];
  va_list args;
  va_start(args, format);
  vsnprintf(ligne, sizeof(ligne), format, args);
  va_end(args);
  Serial.println(ligne);
  reseauDiffuser(ligne, strlen(ligne));
}

static void envoyerJson() {
  lgTrame = 0;
  ajouter("[DATA]{\"t\":%lu,\"hz\":%lu,\"tof\":[", millis(), boucleHz);
  for (uint8_t i = 0; i < 4; i++) {
    const CapteurToF &t = tof[i];
    ajouter("%s{\"n\":\"%s\",\"m\":\"%s\",\"d\":%d,\"s\":\"%s\",\"p\":%d",
            i ? "," : "", t.nom, t.modele, t.brut, distanceCode(i), t.porteeMm);
    if (i == CANAL_AVANT && t.present)
      ajouter(",\"st\":\"%s\"", distanceStatutAvant());
    ajouter("}");
  }

  ajouter("],\"col\":[");
  for (int i = 0; i < 3; i++) {
    const CapteurCouleur &k = couleurs[i];
    ajouter("%s{\"n\":\"%s\",\"ok\":%d", i ? "," : "", k.nom,
            k.present ? (k.valide ? 1 : 0) : -1);
    if (k.valide)
      ajouter(",\"r\":%u,\"g\":%u,\"b\":%u,\"c\":%u,\"lum\":%.3f,"
              "\"lux\":%.0f,\"k\":%d,\"sat\":%d,\"nom\":\"%s\"",
              k.r, k.g, k.b, k.c, k.lum, k.lux, k.cct, k.sature, k.couleur);
    ajouter("}");
  }

  ajouter("],\"imu\":");
  if (gyroPresent()) {
    const GyroData &g = gyroBrut();
    const AccelData &a = gyroAccel();
    ajouter("{\"cap\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
            "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"temp\":%.1f,\"biais\":%.4f}",
            gyroCap(), g.gyroX, g.gyroY, gyroVitesseZ(),
            a.accelX, a.accelY, a.accelZ, gyroTemperature(), gyroBiaisZ());
  } else {
    ajouter("null");
  }

  ajouter(",\"mot\":{\"stby\":%d,\"m\":[", moteursActive());
  for (int i = 0; i < 4; i++) {
    ajouter("%s{\"cmd\":%d,\"v\":%.0f,\"enc\":%ld}", i ? "," : "",
            moteurs[i].consigne, moteurs[i].vitesse, moteurCompte(i));
  }

  ajouter("]},\"net\":{\"ok\":%d,\"ip\":\"%s\",\"rssi\":%d,\"clients\":%d}}",
          reseauConnecte(), reseauIp().c_str(), reseauRssi(), reseauNbClients());

  Serial.println(trame);
  reseauDiffuser(trame, lgTrame);
}

void telemetrieDemarrer() {
  dernierCalculHz = millis();
  nbBoucles = 0;
}

void telemetrieMettreAJour() {
  nbBoucles++;
  unsigned long maintenant = millis();
  if (maintenant - dernierCalculHz >= 1000) {
    boucleHz = nbBoucles * 1000UL / (maintenant - dernierCalculHz);
    nbBoucles = 0;
    dernierCalculHz = maintenant;
  }
  if (PERIODE_JSON_MS && maintenant - dernierJson >= PERIODE_JSON_MS) {
    dernierJson = maintenant;
    envoyerJson();
  }
  if (PERIODE_TEXTE_MS && maintenant - dernierTexte >= PERIODE_TEXTE_MS) {
    dernierTexte = maintenant;
    afficherTexte();
  }
}
