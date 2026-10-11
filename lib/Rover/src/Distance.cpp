#include "Distance.h"

#include <Wire.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include "Bus.h"
#include "Config.h"

static VL53L1X tofAvant;
static VL53L0X tofArriere, tofDroite, tofGauche;

// Etat au 01.09.26 (Suivi n.52 et 54) : le VL53L0X droit est
// DEFAILLANT (renvoie 8191, ne reagit qu'a moins de 10,5 cm).
// Passer fiable a true une fois remplace par celui de reserve.
CapteurToF tof[4] = {
  { "avant",   "VL53L1X", 4000, true,  false, -1, 0, M_JAMAIS_LU, 0 },
  { "arriere", "VL53L0X", 1200, true,  false, -1, 0, M_JAMAIS_LU, 0 },
  { "droite",  "VL53L0X", 1200, false, false, -1, 0, M_JAMAIS_LU, 0 },
  { "gauche",  "VL53L0X",  600, true,  false, -1, 0, M_JAMAIS_LU, 0 }
};

static VL53L0X& capteurL0X(uint8_t canal) {
  return (canal == CANAL_ARRIERE) ? tofArriere
       : (canal == CANAL_DROITE)  ? tofDroite : tofGauche;
}

static void initVL53L0X(uint8_t canal) {
  VL53L0X &s = capteurL0X(canal);
  muxChoisir(canal);
  s.setBus(&Wire);
  s.setTimeout(200);
  if (s.init()) {
    s.startContinuous();
    tof[canal].present = true;
    tof[canal].derniereMs = millis();
  }
}

void distanceInit() {
  muxChoisir(CANAL_AVANT);
  tofAvant.setBus(&Wire);
  tofAvant.setTimeout(200);
  if (tofAvant.init()) {
    tofAvant.setDistanceMode(VL53L1X::Long);
    tofAvant.setMeasurementTimingBudget(33000);   // minimum en mode Long
    tofAvant.startContinuous(33);                 // ~30 mesures/s
    tof[CANAL_AVANT].present = true;
    tof[CANAL_AVANT].derniereMs = millis();
  }
  initVL53L0X(CANAL_ARRIERE);
  initVL53L0X(CANAL_DROITE);
  initVL53L0X(CANAL_GAUCHE);
}

static void lireCapteur(uint8_t canal) {
  CapteurToF &t = tof[canal];
  if (!t.present) { t.etat = M_ABSENT; return; }

  muxChoisir(canal);
  int d = -1;
  bool pret;
  if (canal == CANAL_AVANT) {
    pret = tofAvant.dataReady();
    if (pret) {
      d = tofAvant.read(false);
      t.statutL1X = tofAvant.ranging_data.range_status;
    }
  } else {
    VL53L0X &s = capteurL0X(canal);
    pret = (s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
    if (pret) d = s.readRangeContinuousMillimeters();   // immediat : mesure prete
  }

  unsigned long maintenant = millis();
  if (!pret) {
    if (maintenant - t.derniereMs > DELAI_MESURE_MAX_MS) t.etat = M_TIMEOUT;
    return;
  }
  t.derniereMs = maintenant;
  t.brut = d;
  if (canal != CANAL_AVANT && d >= VL53L0X_CODE_ERREUR) t.etat = M_HORS_PORTEE;
  else t.etat = M_OK;
}

void distanceMettreAJour() {
  for (uint8_t c = 0; c < 4; c++) lireCapteur(c);
}

void distanceAttendrePremiereMesure(unsigned long delaiMs) {
  unsigned long debut = millis();
  while (millis() - debut < delaiMs) {
    distanceMettreAJour();
    bool tous = true;
    for (auto &t : tof)
      if (t.present && t.etat == M_JAMAIS_LU) tous = false;
    if (tous) return;
    delay(2);
  }
}

int distanceUtile(uint8_t canal) {
  const CapteurToF &t = tof[canal];
  if (!t.fiable) return -1;
  if (t.etat == M_OK) return t.brut;
  if (t.etat == M_HORS_PORTEE) return t.porteeMm;
  return -1;
}

const char* distanceCode(uint8_t canal) {
  const CapteurToF &t = tof[canal];
  if (t.etat == M_ABSENT) return "abs";
  if (!t.fiable) return "nf";
  switch (t.etat) {
    case M_OK:          return "ok";
    case M_HORS_PORTEE: return "hp";
    case M_TIMEOUT:     return "err";
    default:            return "--";
  }
}

const char* distanceStatutAvant() {
  return VL53L1X::rangeStatusToString((VL53L1X::RangeStatus) tof[CANAL_AVANT].statutL1X);
}

void distanceTexte(uint8_t canal, char* buf, size_t taille, bool lisible) {
  const CapteurToF &t = tof[canal];
  const char* nf = t.fiable ? "" : "NF:";
  const char* mm = lisible ? " mm" : "";
  switch (t.etat) {
    case M_ABSENT:    snprintf(buf, taille, lisible ? "ABSENT" : "ABS"); return;
    case M_JAMAIS_LU: snprintf(buf, taille, "%s--", nf); return;
    case M_TIMEOUT:   snprintf(buf, taille, "%sERR", nf); return;
    case M_HORS_PORTEE:
      if (t.fiable) snprintf(buf, taille, ">%d%s", t.porteeMm, mm);
      else          snprintf(buf, taille, "NF:%d", t.brut);
      return;
    case M_OK:
      if (!lisible && canal == CANAL_AVANT && t.statutL1X != VL53L1X::RangeValid)
        snprintf(buf, taille, "%s%d(%s)", nf, t.brut, distanceStatutAvant());
      else
        snprintf(buf, taille, "%s%d%s", nf, t.brut, t.fiable ? mm : "");
      return;
  }
}
