#include "RelaisMaj.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include "Enregistrement.h"
#include "LiaisonRover.h"

static const size_t TAILLE_MAX = 3 * 1024 * 1024;        // partition programme du rover
static const unsigned long ATTENTE_BLOC_MS = 5000;

static const char* motDePasse = nullptr;
static volatile bool enCours = false;

void relaisMajInit(const char* motDePasseOta) { motDePasse = motDePasseOta; }
bool relaisMajEnCours() { return enCours; }

// CRC-32 standard (le meme que zlib.crc32 en Python)
static uint32_t crc32(const uint8_t* d, size_t n) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

static bool recevoir(uint8_t* tampon, size_t taille) {
  size_t recu = 0;
  while (recu < taille) {
    size_t bloc = min(TAILLE_BLOC_MAJ, taille - recu);
    size_t dansBloc = 0;
    unsigned long debut = millis();
    while (dansBloc < bloc) {
      int n = Serial.available();
      if (n > 0) {
        dansBloc += Serial.readBytes(tampon + recu + dansBloc, min((size_t) n, bloc - dansBloc));
        debut = millis();
      } else if (millis() - debut > ATTENTE_BLOC_MS) {
        Serial.printf("\n[MAJ] ERREUR : plus de donnees apres %u octets\n", (unsigned) (recu + dansBloc));
        return false;
      } else {
        delay(1);
      }
    }
    recu += bloc;
    Serial.printf("[MAJ] ACK %u\n", (unsigned) recu);
  }
  return true;
}

static IPAddress adresseRover() {
  IPAddress ip = liaisonRoverIp();
  if (ip == IPAddress(0, 0, 0, 0)) ip = MDNS.queryHost("rover", 2000);
  return ip;
}

static void envoyerAuRover(uint8_t* programme, size_t taille) {
  IPAddress ip = adresseRover();
  if (ip == IPAddress(0, 0, 0, 0)) {
    Serial.println("[MAJ] ERREUR : rover introuvable sur le WiFi");
    return;
  }
  Serial.printf("[MAJ] envoi au rover (%s), %u octets...\n", ip.toString().c_str(), (unsigned) taille);
  liaisonRoverActiver(false);          // libere le WiFi le temps de l'envoi

  HTTPClient http;
  http.setTimeout(60000);
  http.begin(String("http://") + ip.toString() + "/maj");
  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("X-Mot-De-Passe", motDePasse ? motDePasse : "");
  unsigned long debut = millis();
  int code = http.POST(programme, taille);
  String reponse = http.getString();
  http.end();

  if (code == 200) {
    Serial.printf("[MAJ] OK en %.1f s : %s\n", (millis() - debut) / 1000.0, reponse.c_str());
  } else {
    Serial.printf("[MAJ] ERREUR rover (HTTP %d) : %s\n", code, reponse.c_str());
  }
  liaisonRoverActiver(true);           // la liaison se refait apres le redemarrage du rover
}

void relaisMajRecevoir(const char* arguments) {
  unsigned long taille = 0, crcAttendu = 0;
  if (sscanf(arguments, " %lu %lx", &taille, &crcAttendu) != 2 || taille == 0 || taille > TAILLE_MAX) {
    Serial.println("[MAJ] ERREUR : en-tete invalide (attendu : !MAJROVER <taille> <crc32>)");
    return;
  }
  uint8_t* programme = (uint8_t*) ps_malloc(taille);
  if (!programme) {
    Serial.println("[MAJ] ERREUR : memoire PSRAM insuffisante");
    return;
  }
  enCours = true;
  enregistrementArreter();             // le WiFi et la carte TF ne servent qu'a ca
  Serial.println("[MAJ] PRET");

  if (recevoir(programme, taille)) {
    uint32_t crc = crc32(programme, taille);
    if (crc != crcAttendu) {
      Serial.printf("[MAJ] ERREUR : CRC %08x, attendu %08lx (transmission abimee)\n", crc, crcAttendu);
    } else {
      Serial.println("[MAJ] CRC OK");
      envoyerAuRover(programme, taille);
    }
  }
  free(programme);
  enCours = false;
}
