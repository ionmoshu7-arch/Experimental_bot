#include "Bus.h"

#include <Wire.h>
#include "Config.h"

static uint8_t canalActif = 0xFF;   // 0xFF = inconnu ou tous fermes

void busInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(100);
}

bool busRepond(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

bool busLireRegistre(uint8_t adresse, uint8_t reg, uint8_t &valeur) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(adresse, (uint8_t) 1) != 1) return false;
  valeur = Wire.read();
  return true;
}

bool muxPresent() {
  return busRepond(ADRESSE_MUX);
}

void muxChoisir(uint8_t canal) {
  if (canal == canalActif) return;   // evite une ecriture I2C inutile
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(1 << canal);
  canalActif = (Wire.endTransmission() == 0) ? canal : 0xFF;
}

void muxFermer() {
  Wire.beginTransmission(ADRESSE_MUX);
  Wire.write(0);
  Wire.endTransmission();
  canalActif = 0xFF;
}
