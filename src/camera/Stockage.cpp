#include "Stockage.h"

#include <FS.h>
#include <SD_MMC.h>

// Brochage TF de la carte (Freenove / Diymore ESP32-S3-WROOM CAM)
static const int SD_CLK = 39, SD_CMD = 38, SD_D0 = 40;
static const char* DOSSIER_PHOTOS = "/photos";

static bool present = false;
static int prochainNumero = 1;
static int nbPhotos = 0;

bool stockageInit() {
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  if (!SD_MMC.begin("/sdcard", true)) {      // true : mode 1 bit
    Serial.println("[SD] pas de carte TF : photos desactivees");
    return present = false;
  }
  if (SD_MMC.cardType() == CARD_NONE) return present = false;
  SD_MMC.mkdir(DOSSIER_PHOTOS);

  // reprend la numerotation apres la derniere photo
  File dossier = SD_MMC.open(DOSSIER_PHOTOS);
  for (File f = dossier.openNextFile(); f; f = dossier.openNextFile()) {
    int n = 0;
    if (sscanf(f.name(), "IMG_%d.jpg", &n) == 1) {
      nbPhotos++;
      if (n >= prochainNumero) prochainNumero = n + 1;
    }
  }
  present = true;
  Serial.printf("[SD] carte TF %llu Mo, %d photo(s), %u Mo libres\n",
                SD_MMC.cardSize() / (1024 * 1024), nbPhotos, stockageLibreMo());
  return true;
}

bool stockagePresent() { return present; }
int stockageNbPhotos() { return nbPhotos; }

uint32_t stockageLibreMo() {
  if (!present) return 0;
  return (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024);
}

String stockageEnregistrerPhoto(const uint8_t* jpeg, size_t taille) {
  if (!present) return "";
  char chemin[32];
  snprintf(chemin, sizeof(chemin), "%s/IMG_%04d.jpg", DOSSIER_PHOTOS, prochainNumero);
  File f = SD_MMC.open(chemin, FILE_WRITE);
  if (!f) return "";
  size_t ecrit = f.write(jpeg, taille);
  f.close();
  if (ecrit != taille) return "";
  prochainNumero++;
  nbPhotos++;
  return String(chemin);
}
