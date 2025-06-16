#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <HoneywellZephyrI2C.h>

// Sensor objects
MAX30105 particleSensor;
ZephyrFlowRateSensor flowSensor(0x49, 50, ZephyrFlowRateSensor::SCCM);

// Heart rate variables (original method)
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// SPO2 variables
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength = 100; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRateAlgo; //heart rate value from algorithm
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte bufferIndex = 0;
bool bufferReady = false;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 40; // 25Hz sampling for SPO2

// Flow rate variables
float currentFlowRate = 0.0;
bool flowSensorReady = false;
unsigned long lastFlowReading = 0;
const unsigned long flowReadInterval = 100; // Read flow every 100ms

// Hardware Serial 2 pins
#define RXD2 16
#define TXD2 17

// Data transmission
unsigned long lastTransmission = 0;
unsigned long transmissionInterval = 1000;

// Combined data structure
#pragma pack(push, 1)
struct CombinedSensorData {
  uint16_t avgBPM;
  uint16_t algoBPM;
  uint16_t spo2Value;
  uint32_t irValue;
  float flowRate;           // Added flow rate
  uint8_t fingerDetected;
  uint8_t validHR;
  uint8_t validSPO2;
  uint8_t flowSensorStatus; // Added flow sensor status
  uint8_t checksum;
} sensorData;
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("Initializing sensors...");

  // Initialize I2C
  Wire.begin(); // Uses default pins GPIO 21 (SDA), GPIO 22 (SCL)

  // Initialize MAX30105 heart rate sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found");
    while (1);
  }
  Serial.println("MAX30105 initialized successfully");

  // Configure MAX30105 for both heart rate and SPO2
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  // Initialize flow sensor
  flowSensor.begin();
  flowSensorReady = true;
  Serial.println("Flow sensor initialized successfully");
  
  Serial.println("Place finger on heart rate sensor");
  
  // Initialize buffers
  for (int i = 0; i < 100; i++) {
    irBuffer[i] = 0;
    redBuffer[i] = 0;
  }
}

void loop() {
  // Read flow sensor data
  readFlowSensor();
  
  // Check if new data is available from heart rate sensor
  if (particleSensor.available()) {
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();
    
    // Original heart rate detection
    checkHeartRate(irValue);
    
    // SPO2 buffer management (time-based sampling)
    if (millis() - lastSampleTime >= sampleInterval) {
      manageSPO2Buffer(irValue, redValue);
      lastSampleTime = millis();
    }
    
    particleSensor.nextSample(); // Move to next sample
    
    updateDataStructure(irValue);
    handleSerialCommunication();
    transmitDataPeriodically();
    printSerialOutput(irValue);
  }
  
  // Check sensor for new data
  particleSensor.check();
}

void readFlowSensor() {
  if (flowSensorReady && (millis() - lastFlowReading >= flowReadInterval)) {
    if (flowSensor.readSensor() == 0) {
      currentFlowRate = flowSensor.flow();
    }
    lastFlowReading = millis();
  }
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

void manageSPO2Buffer(long irValue, long redValue) {
  // Store values in buffer
  irBuffer[bufferIndex] = irValue;
  redBuffer[bufferIndex] = redValue;
  bufferIndex++;

  // Initial buffer fill (first 100 samples)
  if (!bufferReady && bufferIndex >= 100) {
    bufferReady = true;
    bufferIndex = 0;
    // Calculate initial SPO2 and HR
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRateAlgo, &validHeartRate);
    Serial.println("SPO2 buffer ready - starting calculations");
  }
  
  // Continuous update (every 25 samples after initial fill)
  if (bufferReady && bufferIndex >= 25) {
    // Shift buffer - move last 75 samples to beginning
    for (int i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
    
    // Reset index to fill last 25 positions
    bufferIndex = 0;
    
    // Recalculate SPO2 and HR
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRateAlgo, &validHeartRate);
  }
}

void updateDataStructure(long irValue) {
  sensorData.avgBPM = (uint16_t)beatAvg;
  sensorData.algoBPM = bufferReady ? (uint16_t)heartRateAlgo : 0;
  sensorData.spo2Value = bufferReady ? (uint16_t)spo2 : 0;
  sensorData.irValue = (uint32_t)irValue;
  sensorData.flowRate = currentFlowRate;
  sensorData.fingerDetected = (irValue >= 50000) ? 1 : 0;
  sensorData.validHR = bufferReady ? (uint8_t)validHeartRate : 0;
  sensorData.validSPO2 = bufferReady ? (uint8_t)validSPO2 : 0;
  sensorData.flowSensorStatus = flowSensorReady ? 1 : 0;
  sensorData.checksum = calculateChecksum();
}

uint8_t calculateChecksum() {
  uint8_t cs = 0;
  uint8_t* data = (uint8_t*)&sensorData;
  for (size_t i = 0; i < sizeof(sensorData) - 1; i++) cs ^= data[i];
  return cs;
}

void handleSerialCommunication() {
  if (Serial2.available()) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();

    if (cmd == "GET_DATA") transmitSensorData();
    else if (cmd == "GET_STATUS") sendStatus();
    else if (cmd.startsWith("SET_INTERVAL:")) setInterval(cmd);
  }
}

void transmitDataPeriodically() {
  if (millis() - lastTransmission >= transmissionInterval) {
    transmitSensorData();
    lastTransmission = millis();
  }
}

void transmitSensorData() {
  Serial2.write(0xFF); // Start marker
  Serial2.write((uint8_t*)&sensorData, sizeof(sensorData));
  Serial2.write(0xFE); // End marker

  Serial.print("Sent - Avg BPM: ");
  Serial.print(sensorData.avgBPM);
  Serial.print(", Flow: ");
  Serial.print(sensorData.flowRate);
  Serial.print(" SCCM");
  
  if (bufferReady) {
    Serial.print(", Algo BPM: ");
    Serial.print(sensorData.algoBPM);
    Serial.print(", SPO2: ");
    Serial.print(sensorData.spo2Value);
    Serial.print("%");
  }
  Serial.println();
}

void sendStatus() {
  String status = "STATUS:HR_SENSOR:" + String(particleSensor.begin() ? "OK" : "ERROR");
  status += ",FLOW_SENSOR:" + String(flowSensorReady ? "OK" : "ERROR");
  status += ",FINGER:" + String(sensorData.fingerDetected ? "YES" : "NO");
  status += ",BUFFER:" + String(bufferReady ? "READY" : "FILLING");
  status += ",FLOW_RATE:" + String(currentFlowRate);
  
  if (bufferReady) {
    status += ",SPO2_VALID:" + String(validSPO2 ? "YES" : "NO");
    status += ",HR_VALID:" + String(validHeartRate ? "YES" : "NO");
  }
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
  Serial.print(", Flow=");
  Serial.print(currentFlowRate);
  Serial.print(" SCCM");
  
  if (bufferReady) {
    Serial.print(", Algo BPM=");
    Serial.print(heartRateAlgo);
    Serial.print(" (valid:");
    Serial.print(validHeartRate);
    Serial.print("), SPO2=");
    Serial.print(spo2);
    Serial.print("% (valid:");
    Serial.print(validSPO2);
    Serial.print(")");
  } else {
    Serial.print(", SPO2 Buffer: ");
    Serial.print(bufferIndex);
    Serial.print("/100");
  }
  
  if (irValue < 50000) {
    Serial.print(" No finger detected!");
  }
  Serial.println();
}
