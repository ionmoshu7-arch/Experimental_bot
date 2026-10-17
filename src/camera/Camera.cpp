#include "Camera.h"

// Brochages connus des cartes ESP32-S3 camera : essayes dans l'ordre
// jusqu'a ce que le capteur reponde.
struct Brochage {
  const char* nom;
  int pwdn, reset, xclk, siod, sioc;
  int y2, y3, y4, y5, y6, y7, y8, y9;
  int vsync, href, pclk;
};
static const Brochage BROCHAGES[] = {
  { "Freenove / ESP32-S3-EYE", -1, -1, 15, 4, 5,   11, 9, 8, 10, 12, 18, 17, 16,   6, 7, 13 },
  { "Seeed XIAO ESP32S3 Sense", -1, -1, 10, 40, 39, 15, 17, 18, 16, 14, 12, 11, 48,  38, 47, 13 },
  { "LilyGO T-Camera S3",      -1, 39, 38, 5, 4,   14, 47, 48, 21, 13, 11, 10, 9,    8, 18, 12 },
};
static const char* brochageTrouve = "aucun";

struct Resolution { const char* nom; framesize_t taille; };
static const Resolution RESOLUTIONS[] = {
  { "QVGA", FRAMESIZE_QVGA },   // 320x240
  { "VGA",  FRAMESIZE_VGA },    // 640x480
  { "SVGA", FRAMESIZE_SVGA },   // 800x600
  { "XGA",  FRAMESIZE_XGA },    // 1024x768
  { "HD",   FRAMESIZE_HD },     // 1280x720
  { "SXGA", FRAMESIZE_SXGA },   // 1280x1024
  { "UXGA", FRAMESIZE_UXGA },   // 1600x1200
};

static volatile uint32_t nbImages = 0;
static uint32_t nbImagesPrec = 0;
static unsigned long dernierCalcul = 0;
static float ips = 0;

static bool essayer(const Brochage &b) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = b.y2; c.pin_d1 = b.y3; c.pin_d2 = b.y4; c.pin_d3 = b.y5;
  c.pin_d4 = b.y6; c.pin_d5 = b.y7; c.pin_d6 = b.y8; c.pin_d7 = b.y9;
  c.pin_xclk = b.xclk;
  c.pin_pclk = b.pclk;
  c.pin_vsync = b.vsync;
  c.pin_href = b.href;
  c.pin_sccb_sda = b.siod;
  c.pin_sccb_scl = b.sioc;
  c.pin_pwdn = b.pwdn;
  c.pin_reset = b.reset;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] brochage %-26s : pas de reponse (0x%x)\n", b.nom, err);
    esp_camera_deinit();
    return false;
  }
  return true;
}

bool cameraInit() {
  for (auto &b : BROCHAGES) {
    if (essayer(b)) {
      brochageTrouve = b.nom;
      Serial.printf("[CAM] brochage %s : camera %s\n", b.nom, cameraModele());
      return true;
    }
  }
  return false;
}

const char* cameraBrochage() { return brochageTrouve; }

const char* cameraModele() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return "absente";
  switch (s->id.PID) {
    case OV2640_PID: return "OV2640";
    case OV3660_PID: return "OV3660";
    case OV5640_PID: return "OV5640";
    default:         return "inconnue";
  }
}

bool cameraResolution(const char* nom) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;
  for (auto &r : RESOLUTIONS)
    if (strcasecmp(nom, r.nom) == 0) return s->set_framesize(s, r.taille) == 0;
  return false;
}

const char* cameraNomResolution() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return "?";
  for (auto &r : RESOLUTIONS)
    if (s->status.framesize == r.taille) return r.nom;
  return "?";
}

bool cameraQualite(int q) {
  sensor_t* s = esp_camera_sensor_get();
  return s && s->set_quality(s, constrain(q, 10, 63)) == 0;
}

int cameraQualiteActuelle() {
  sensor_t* s = esp_camera_sensor_get();
  return s ? s->status.quality : 0;
}

void cameraRetourner(bool vertical, bool miroir) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  s->set_vflip(s, vertical);
  s->set_hmirror(s, miroir);
}

bool cameraRetourneeV() {
  sensor_t* s = esp_camera_sensor_get();
  return s && s->status.vflip;
}

bool cameraMiroir() {
  sensor_t* s = esp_camera_sensor_get();
  return s && s->status.hmirror;
}

void cameraCompterImage() { nbImages++; }

float cameraImagesParSeconde() {
  unsigned long maintenant = millis();
  if (maintenant - dernierCalcul >= 1000) {
    uint32_t n = nbImages;
    ips = (n - nbImagesPrec) * 1000.0 / (maintenant - dernierCalcul);
    nbImagesPrec = n;
    dernierCalcul = maintenant;
  }
  return ips;
}
