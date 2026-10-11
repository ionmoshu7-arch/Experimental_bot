/*
 * Gyro.h - MPU-6500 (FastIMU), cap integre sur l'axe Z
 * ------------------------------------------------------------
 * Convention validee : rotation a DROITE = cap NEGATIF.
 */
#pragma once

#include <Arduino.h>
#include "FastIMU.h"

bool gyroInit();               // +/-250 deg/s ; false si absent
bool gyroPresent();
void gyroCalibrer();           // 1,5 s immobile : mesure le biais Z, cap a 0
void gyroMettreAJour();        // a appeler a chaque tour de boucle

float gyroCap();
void gyroRemettreCap();
float gyroBiaisZ();
float gyroVitesseZ();          // deg/s, biais retire
const GyroData& gyroBrut();
const AccelData& gyroAccel();
float gyroTemperature();
