#include "Couleur.h"

#include <Wire.h>
#include "Bus.h"
#include "Config.h"
#include <Preferences.h>

// Temps d'integration = (256 - ATIME) x 2,4 ms
static const uint8_t TCS_ATIME = 0xF6;                // 10 cycles = 24 ms (~40 mesures/s)
static const uint8_t TCS_GAIN = 0x02;                 // 0=1x 1=4x 2=16x 3=60x
static const uint8_t TCS_GAINS[4] = { 1, 4, 16, 60 };

// Classement approximatif de la couleur du sol : a ajuster selon
// l'eclairage et la hauteur des capteurs (voir lum).
static const float SEUIL_NOIR = 0.03;    // luminosite relative en dessous = noir
static const float SEUIL_BLANC = 0.30;   // au dessus et sans dominante = blanc
static const float SEUIL_DOMINANTE = 0.42;

CapteurCouleur couleurs[3] = {
  { "sol_gauche", CANAL_SOL_GAUCHE },
  { "sol_centre", CANAL_SOL_CENTRE },
  { "sol_droit",  CANAL_SOL_DROIT },
};

static bool tcsEcrire(uint8_t reg, uint8_t valeur) {
  Wire.beginTransmission(ADRESSE_TCS);
  Wire.write(0x80 | reg);
  Wire.write(valeur);
  return Wire.endTransmission() == 0;
}

static bool tcsLire(uint8_t reg, uint8_t* dest, uint8_t n) {
  Wire.beginTransmission(ADRESSE_TCS);
  Wire.write(0xA0 | reg);          // commande + auto-increment
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(ADRESSE_TCS, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dest[i] = Wire.read();
  return true;
}

static uint16_t comptageMax() {
  uint32_t m = (256 - TCS_ATIME) * 1024UL;
  return m > 65535 ? 65535 : m;
}

static void initCapteur(CapteurCouleur &k) {
  muxChoisir(k.canal);
  uint8_t id = 0;
  // 0x44 = TCS34721/34725, 0x4D = TCS34723/34727
  if (!tcsLire(0x12, &id, 1) || (id != 0x44 && id != 0x4D)) return;
  tcsEcrire(0x01, TCS_ATIME);
  tcsEcrire(0x0F, TCS_GAIN);
  tcsEcrire(0x00, 0x01);           // PON
  delay(3);
  tcsEcrire(0x00, 0x03);           // PON + AEN
  k.present = true;
  k.couleur = "--";
}

static const char* classer(const CapteurCouleur &k) {
  if (k.sature) return "sature";
  if (k.lum < SEUIL_NOIR) return "noir";
  float somme = (float) k.r + k.g + k.b;
  if (somme <= 0) return "noir";
  float fr = k.r / somme, fg = k.g / somme, fb = k.b / somme;
  if (fr > SEUIL_DOMINANTE && fg > 0.30) return "jaune";
  if (fr > SEUIL_DOMINANTE) return "rouge";
  if (fg > SEUIL_DOMINANTE) return "vert";
  if (fb > SEUIL_DOMINANTE) return "bleu";
  return k.lum > SEUIL_BLANC ? "blanc" : "gris";
}

static void lireCapteur(CapteurCouleur &k) {
  if (!k.present) return;
  muxChoisir(k.canal);

  uint8_t statut = 0;
  if (!tcsLire(0x13, &statut, 1) || !(statut & 0x01)) return;  // AVALID

  uint8_t d[8];
  if (!tcsLire(0x14, d, 8)) return;
  k.c = d[0] | (d[1] << 8);
  k.r = d[2] | (d[3] << 8);
  k.g = d[4] | (d[5] << 8);
  k.b = d[6] | (d[7] << 8);
  k.valide = true;

  uint16_t maxi = comptageMax();
  k.lum = (float) k.c / maxi;
  k.sature = k.c >= maxi;

  // Lux et temperature de couleur : methode AMS DN40
  float ir = (k.r + k.g + k.b > k.c) ? (k.r + k.g + k.b - k.c) / 2.0 : 0;
  float r2 = k.r - ir, g2 = k.g - ir, b2 = k.b - ir;
  float atimeMs = (256 - TCS_ATIME) * 2.4;
  float cpl = atimeMs * TCS_GAINS[TCS_GAIN] / 310.0;
  k.lux = (0.136 * r2 + g2 - 0.444 * b2) / cpl;
  if (k.lux < 0) k.lux = 0;
  k.cct = (r2 > 0) ? (int) (3810.0 * b2 / r2 + 1391) : 0;

  k.couleur = classer(k);
}

// Capteurs eteints : veille (ENABLE = 0), plus aucune lecture I2C.
// Leur LED blanche reste allumee : sa broche n'est pas reliee a une sortie.
static bool actifs = true;

void couleurInit() {
  for (auto &k : couleurs) initCapteur(k);
  Preferences p;
  p.begin("couleur", true);
  bool memorise = p.getBool("actifs", true);
  p.end();
  if (!memorise) couleurActiver(false);
}

void couleurActiver(bool actif) {
  actifs = actif;
  for (auto &k : couleurs) {
    if (!k.present) continue;
    muxChoisir(k.canal);
    if (actif) {
      tcsEcrire(0x00, 0x01);             // PON
      delay(3);
      tcsEcrire(0x00, 0x03);             // PON + AEN
    } else {
      tcsEcrire(0x00, 0x00);             // veille
      k.valide = false;                  // plus de mesure fraiche
      k.couleur = "eteint";
    }
  }
  Preferences p;
  p.begin("couleur", false);
  p.putBool("actifs", actif);
  p.end();
}

bool couleurActifs() { return actifs; }

void couleurMettreAJour() {
  if (!actifs) return;
  for (auto &k : couleurs) lireCapteur(k);
}
