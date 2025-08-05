#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// Heart rate variables
const int BUFFER_SIZE = 100;  // Buffer for IR samples
long irBuffer[BUFFER_SIZE];   // Raw IR values
int bufferIndex = 0;          // Current position in buffer
float bpm = 0;                // Calculated heart rate
bool beatDetected = false;    // Flag for heartbeat detection
unsigned long lastBeatTime = 0; // Timestamp of last beat

// Thresholds and settings
#define IR_THRESHOLD 50000    // Adjust based on your signal strength
#define MIN_BEAT_INTERVAL 200 // Minimum time between beats (ms)
#define MAX_BEAT_INTERVAL 1200 // Maximum time between beats (ms)

void setup() {
  Serial.begin(115200);
  Serial.println("MAX30102 Heart Rate Monitor");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    while (1);
  }

  // Configure sensor (optimized for heart rate)
  byte ledBrightness = 0x1F;  // Medium brightness
  byte sampleAverage = 4;     // Smoothing
  byte ledMode = 2;           // Red + IR
  int sampleRate = 400;       // 400 samples/sec
  int pulseWidth = 411;       // Long pulse for better sensitivity
  int adcRange = 4096;        // Medium ADC range

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop() {
  // Read IR value
  long irValue = particleSensor.getIR();

  // Store in buffer (for rolling average)
  irBuffer[bufferIndex] = irValue;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  // Simple beat detection (adjust threshold as needed)
  if (irValue > IR_THRESHOLD && !beatDetected) {
    unsigned long currentTime = millis();
    unsigned long beatInterval = currentTime - lastBeatTime;

    // Validate beat interval (avoid false positives)
    if (beatInterval > MIN_BEAT_INTERVAL && beatInterval < MAX_BEAT_INTERVAL) {
      bpm = 60000.0 / beatInterval;  // Convert ms to BPM
      Serial.print("♥ Heartbeat! BPM: ");
      Serial.println(bpm);
      beatDetected = true;
    }
    lastBeatTime = currentTime;
  }

  // Reset beat detection if signal drops
  if (irValue < IR_THRESHOLD && beatDetected) {
    beatDetected = false;
  }

  delay(10);  // Small delay to stabilize readings
}