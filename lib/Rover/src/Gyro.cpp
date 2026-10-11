#include "Gyro.h"

#include "Config.h"

static MPU6500 IMU;
static calData calib = { 0 };
static bool present = false;
static GyroData gyro;
static AccelData accel;
static float temperature = 0;
static float biaisZ = 0;
static float cap = 0;
static unsigned long dernierEchantillon = 0;

bool gyroInit() {
  present = (IMU.init(calib, ADRESSE_IMU) == 0);
  if (present) IMU.setGyroRange(250);
  dernierEchantillon = micros();
  return present;
}

bool gyroPresent() { return present; }

void gyroCalibrer() {
  if (!present) return;
  Serial.println("Calibration du gyroscope - NE PAS BOUGER LE ROVER...");
  double somme = 0;
  for (int i = 0; i < 1500; i++) {
    IMU.update();
    IMU.getGyro(&gyro);
    somme += gyro.gyroZ;
    delay(1);
  }
  biaisZ = somme / 1500.0;
  cap = 0;
  dernierEchantillon = micros();
  Serial.printf("Biais axe Z : %.4f deg/s\n", biaisZ);
}

void gyroMettreAJour() {
  unsigned long maintenant = micros();
  float dt = (maintenant - dernierEchantillon) / 1000000.0;
  dernierEchantillon = maintenant;
  if (!present) return;

  IMU.update();
  IMU.getGyro(&gyro);
  IMU.getAccel(&accel);
  temperature = IMU.getTemp();
  cap += (gyro.gyroZ - biaisZ) * dt;
}

float gyroCap()                { return cap; }
void gyroRemettreCap()         { cap = 0; }
float gyroBiaisZ()             { return biaisZ; }
float gyroVitesseZ()           { return gyro.gyroZ - biaisZ; }
const GyroData& gyroBrut()     { return gyro; }
const AccelData& gyroAccel()   { return accel; }
float gyroTemperature()        { return temperature; }
