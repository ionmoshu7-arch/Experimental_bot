/*
 * Moteurs.h - 4 moteurs TT (2 x TB6612FNG) et leurs encodeurs
 * ------------------------------------------------------------
 * Index 0..3 = M1..M4. Cote droit : M1 (avant), M3 (arriere).
 * Cote gauche : M2 (avant), M4 (arriere).
 * Consigne de -255 a 255, positif = la roue fait avancer le rover.
 */
#pragma once

#include <Arduino.h>

enum IndexMoteur : uint8_t { M1 = 0, M2 = 1, M3 = 2, M4 = 3 };

struct Moteur {
  uint8_t pwm, in1, in2, canalPwm;
  const char* nom;
  uint8_t encA, encB;
  volatile long compte;
  volatile long compteTotal;   // jamais remis a zero (odometrie)
  volatile uint8_t etatPrec;
  int consigne;
  long comptePrec;
  float vitesse;     // mm/s mesuree, filtree
};

extern Moteur moteurs[4];

void moteursInit();                    // moteurs arretes, STBY a LOW
void moteurAppliquer(uint8_t i, int v);
void moteursStop();                    // consignes a 0, STBY inchange
void moteursActiver(bool actif);       // STBY des 2 TB6612FNG
bool moteursActive();
bool moteursActifs();                  // au moins une consigne non nulle

void moteursCotes(int droite, int gauche);   // M1+M3 / M2+M4
void avancer(int v);
void reculer(int v);
void tournerDroite(int v);             // sur place : droite recule, gauche avance
void tournerGauche(int v);

long moteurCompte(uint8_t i);          // lecture protegee des interruptions
long moteurCompteTotal(uint8_t i);     // depuis le demarrage, jamais remis a zero
long compteMoyen();                    // moyenne des valeurs absolues
void moteursRemettreAZero();

// A appeler souvent : recalcule les vitesses toutes les 100 ms
void moteursCalculerVitesses();
