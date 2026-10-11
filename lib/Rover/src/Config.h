/*
 * Config.h - brochage et constantes materielles du rover
 * ------------------------------------------------------------
 * Source unique pour tous les programmes du projet.
 * References :
 *   - Rover_Cablage_Moteurs.pdf (01.09.26)
 *   - 01_Rover_Suivi.xlsx : feuilles Connexions et Calibration
 *
 * PAS DE LED DE CARTE : la LED RGB de la DevKitC-1 est sur GPIO48
 * (v1.0) ou GPIO38 (v1.1). GPIO48 = encodeur M3 phase B et
 * GPIO38 = SDA : l'utiliser fausserait l'encodeur ou le bus I2C.
 * ------------------------------------------------------------
 */
#pragma once

#include <Arduino.h>

// ================== BUS I2C ==================

constexpr uint8_t SDA_PIN = 38;
constexpr uint8_t SCL_PIN = 39;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

constexpr uint8_t ADRESSE_MUX = 0x73;     // TCA9548A, A0/A1 soudes sur VCC
constexpr uint8_t ADRESSE_IMU = 0x68;     // MPU-6500, AD0 a la masse
constexpr uint8_t ADRESSE_TCS = 0x29;     // TCS34725, derriere le mux

// Canaux du TCA9548A
constexpr uint8_t CANAL_AVANT   = 0;      // VL53L1X
constexpr uint8_t CANAL_ARRIERE = 1;      // VL53L0X
constexpr uint8_t CANAL_DROITE  = 2;      // VL53L0X
constexpr uint8_t CANAL_GAUCHE  = 3;      // VL53L0X
constexpr uint8_t CANAL_SOL_GAUCHE = 4;   // TCS34725
constexpr uint8_t CANAL_SOL_CENTRE = 5;   // TCS34725
constexpr uint8_t CANAL_SOL_DROIT  = 6;   // TCS34725

// ================== MOTEURS (2 x TB6612FNG) ==================
// Avant = IN1 HIGH, IN2 LOW. Les inversions de cablage sont deja
// appliquees ici : IN1/IN2 de M1 et M4 echanges (moteurs a l'envers).

constexpr uint8_t STBY_PIN = 16;          // commun aux 2 modules

constexpr uint8_t M1_PWM = 2,  M1_IN1 = 4,  M1_IN2 = 1;    // avant-droit (inverse)
constexpr uint8_t M2_PWM = 10, M2_IN1 = 11, M2_IN2 = 12;   // avant-gauche
constexpr uint8_t M3_PWM = 5,  M3_IN1 = 6,  M3_IN2 = 7;    // arriere-droit
constexpr uint8_t M4_PWM = 13, M4_IN1 = 15, M4_IN2 = 14;   // arriere-gauche (inverse)

constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 8;

// ================== ENCODEURS ==================
// Phases A/B de M2 et M4 echangees : positif = le rover avance.

constexpr uint8_t ENC1_A = 40, ENC1_B = 41;
constexpr uint8_t ENC2_A = 19, ENC2_B = 18;
constexpr uint8_t ENC3_A = 47, ENC3_B = 48;
constexpr uint8_t ENC4_A = 17, ENC4_B = 21;

// ================== CALIBRATION ==================

constexpr float IMPULSIONS_PAR_TOUR = 576.0;   // 3 x 48 x 4
constexpr float DIAMETRE_ROUE_CM = 6.7;
constexpr float CIRCONFERENCE_CM = DIAMETRE_ROUE_CM * 3.14159265;
constexpr float MM_PAR_IMPULSION = CIRCONFERENCE_CM * 10.0 / IMPULSIONS_PAR_TOUR;
