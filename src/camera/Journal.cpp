#include "Journal.h"

#include <FS.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <vector>
#include <algorithm>
#include "Stockage.h"

static const char* DOSSIER = "/journal";
static const unsigned long PERIODE_VIDAGE_MS = 2000;   // ecriture effective sur la carte

static File fichier;
static char nomFichier[40] = "";
static uint32_t demarrage = 0;
static int numero = 0;
static unsigned long debutFichier = 0, dernierVidage = 0;
static SemaphoreHandle_t verrou = nullptr;

struct InfoFichier { String chemin; size_t taille; };

static std::vector<InfoFichier> lister() {
  std::vector<InfoFichier> liste;
  File d = SD_MMC.open(DOSSIER);
  if (!d) return liste;
  for (File f = d.openNextFile(); f; f = d.openNextFile())
    if (!f.isDirectory()) liste.push_back({ String(DOSSIER) + "/" + f.name(), f.size() });
  d.close();
  // noms J<demarrage 5 chiffres>_<n 4 chiffres> : l'ordre alphabetique est chronologique
  std::sort(liste.begin(), liste.end(),
            [](const InfoFichier &a, const InfoFichier &b) { return a.chemin < b.chemin; });
  return liste;
}

// Efface les plus anciens fichiers au-dela de 48 h ou de 2 Go
static void nettoyer() {
  auto liste = lister();
  uint64_t total = 0;
  for (auto &f : liste) total += f.taille;
  size_t i = 0;
  while (i < liste.size() && liste[i].chemin != nomFichier &&
         ((int) (liste.size() - i) > NB_FICHIERS_JOURNAL_MAX || total > TAILLE_JOURNAL_MAX)) {
    total -= liste[i].taille;
    SD_MMC.remove(liste[i].chemin);
    i++;
  }
  if (i) Serial.printf("[JOURNAL] %u ancien(s) fichier(s) efface(s)\n", (unsigned) i);
}

static void nouveauFichier() {
  if (fichier) fichier.close();
  numero++;
  snprintf(nomFichier, sizeof(nomFichier), "%s/J%05u_%04d.jsonl", DOSSIER, (unsigned) demarrage, numero);
  fichier = SD_MMC.open(nomFichier, FILE_WRITE);
  debutFichier = millis();
  nettoyer();
}

void journalInit() {
  verrou = xSemaphoreCreateMutex();
  if (!stockagePresent()) return;
  Preferences prefs;
  prefs.begin("journal", false);
  demarrage = prefs.getUInt("demarrage", 0) + 1;
  prefs.putUInt("demarrage", demarrage);
  prefs.end();
  SD_MMC.mkdir(DOSSIER);
  nouveauFichier();
  Serial.printf("[JOURNAL] %s (un fichier toutes les 10 min, 48 h gardees)\n", nomFichier);
}

void journalMessage(const uint8_t* texte, size_t longueur) {
  if (!verrou || !fichier) return;
  xSemaphoreTake(verrou, portMAX_DELAY);
  if (fichier) {
    char temps[16];
    int n = snprintf(temps, sizeof(temps), "%lu\t", millis());
    fichier.write((const uint8_t*) temps, n);
    fichier.write(texte, longueur);
    fichier.write((const uint8_t*) "\n", 1);
  }
  xSemaphoreGive(verrou);
}

void journalMettreAJour() {
  if (!verrou || !stockagePresent()) return;
  unsigned long maintenant = millis();
  if (maintenant - dernierVidage < PERIODE_VIDAGE_MS) return;
  dernierVidage = maintenant;
  xSemaphoreTake(verrou, portMAX_DELAY);
  if (fichier) fichier.flush();
  if (maintenant - debutFichier >= DUREE_FICHIER_JOURNAL_MS) nouveauFichier();
  xSemaphoreGive(verrou);
}

int journalVider() {
  if (!verrou || !stockagePresent()) return 0;
  xSemaphoreTake(verrou, portMAX_DELAY);
  if (fichier) fichier.close();
  int n = stockageViderDossier(DOSSIER);
  numero = 0;
  nouveauFichier();
  xSemaphoreGive(verrou);
  return n;
}

void journalListeJson(String &sortie) {
  sortie = "[";
  if (verrou && stockagePresent()) {
    xSemaphoreTake(verrou, portMAX_DELAY);
    if (fichier) fichier.flush();
    auto liste = lister();
    xSemaphoreGive(verrou);
    for (size_t i = 0; i < liste.size(); i++) {
      if (i) sortie += ",";
      sortie += "{\"f\":\"" + liste[i].chemin + "\",\"ko\":" + String((unsigned) (liste[i].taille / 1024)) + "}";
    }
  }
  sortie += "]";
}

const char* journalFichierCourant() { return nomFichier; }
