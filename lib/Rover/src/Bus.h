/*
 * Bus.h - bus I2C et multiplexeur TCA9548A
 */
#pragma once

#include <Arduino.h>

void busInit();
bool busRepond(uint8_t adresse);
bool busLireRegistre(uint8_t adresse, uint8_t reg, uint8_t &valeur);

bool muxPresent();
void muxChoisir(uint8_t canal);   // ne reecrit pas si le canal est deja actif
void muxFermer();                 // ferme tous les canaux
