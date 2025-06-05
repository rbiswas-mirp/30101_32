#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

// Heart rate variables
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Hardware Serial 2 pins
#define RXD2 16
#define TXD2 17

// Data transmission
unsigned long lastTransmission = 0;
unsigned long transmissionInterval = 1000;

#pragma pack(push, 1)
struct HeartRateData {
  uint16_t avgBPM;
  uint32_t irValue;
  uint8_t fingerDetected;
  uint8_t checksum;
} hrData;
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("Place finger on sensor");
}

void loop() {
  long irValue = particleSensor.getIR();
  checkHeartRate(irValue);
  updateDataStructure(irValue);
  handleSerialCommunication();
  transmitDataPeriodically();
  printSerialOutput(irValue); // <-- Print output every loop
}

void checkHeartRate(long irValue) {
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}

void updateDataStructure(long irValue) {
  hrData.avgBPM = (uint16_t)beatAvg;
  hrData.irValue = (uint32_t)irValue;
  hrData.fingerDetected = (irValue >= 50000) ? 1 : 0;
  hrData.checksum = calculateChecksum();
}

uint8_t calculateChecksum() {
  uint8_t cs = 0;
  uint8_t* data = (uint8_t*)&hrData;
  for (size_t i = 0; i < sizeof(hrData) - 1; i++) cs ^= data[i];
  return cs;
}

void handleSerialCommunication() {
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();

    if (cmd == "GET_DATA") transmitHeartRateData();
    else if (cmd == "GET_STATUS") sendStatus();
    else if (cmd.startsWith("SET_INTERVAL:")) setInterval(cmd);
  }
}

void transmitDataPeriodically() {
  if (millis() - lastTransmission >= transmissionInterval) {
    transmitHeartRateData();
    lastTransmission = millis();
  }
}

void transmitHeartRateData() {
  Serial2.write(0xFF); // Start marker
  Serial2.write((uint8_t*)&hrData, sizeof(hrData));
  Serial2.write(0xFE); // End marker

  Serial.print("Sent Avg BPM: ");
  Serial.println(hrData.avgBPM);
}

void sendStatus() {
  String status = "STATUS:" + String(particleSensor.begin() ? "OK" : "ERROR");
  status += ",FINGER:" + String(hrData.fingerDetected ? "YES" : "NO");
  Serial2.println(status);
}

void setInterval(String cmd) {
  unsigned long newInterval = cmd.substring(13).toInt();
  if (newInterval >= 100 && newInterval <= 10000) {
    transmissionInterval = newInterval;
    Serial2.print("INTERVAL_SET:");
    Serial2.println(newInterval);
  }
}

void printSerialOutput(long irValue) {
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue < 50000) {
    Serial.print(" No finger detected!");
    Serial2.println("No finger detected!");
  } else {
    Serial2.println("Finger detected.");
  }
  Serial.println();
}
