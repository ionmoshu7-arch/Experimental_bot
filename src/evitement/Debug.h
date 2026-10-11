/*
 * Debug.h - ligne de debug periodique "[DBG] cle=valeur ..."
 * ------------------------------------------------------------
 * Distances en mm, derniere valeur lue : ">1200" = rien dans la
 * portee, ERR = plus de mesure, ABS = absent, -- = pas encore lu,
 * NF: = capteur non fiable (valeur brute, ignoree pour decider).
 * Vitesses par roue en mm/s, positif = vers l'avant.
 * Lue aussi par web/rover_moniteur.html.
 */
#pragma once

void debugDemarrer();
void debugEnvoyer();   // a appeler a chaque tour : n'ecrit qu'a chaque periode
