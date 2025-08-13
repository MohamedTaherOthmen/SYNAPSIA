#include <Wire.h>
#include "MAX30105.h" 

MAX30105 synapsia;

#define WINDOW_SIZE 100
#define SAMPLE_MS 10
#define DEBOUNCE_WINDOWS 3

float irWindow[WINDOW_SIZE];
unsigned short index = 0;
bool windowArrayFilled = false;

float ambientDC = 0.0;
float dcThreshold = 0.0;
float acThreshold = 200.0; 

unsigned short aboveCount = 0;
unsigned belowCount = 0;
bool fingerPresent = false;

long readIR() {
  return synapsia.getIR(); 
}

//--- Compute average (DC) and peak-to-peak (AC) in the window ---
void computeWindowStats(float& meanOut, float& p2pOut) {
  int n = windowArrayFilled ? WINDOW_SIZE : index;
  if (n == 0) { 
    meanOut = 0;
    p2pOut = 0;
    return;
  }
  float sum = 0;
  float minVal = 1e12, maxVal = -1e12;
  for(unsigned short i = 0; i < n; i++){
    float v = irWindow[i];
    sum += v;
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
  }
  meanOut = sum / n;                
  p2pOut = maxVal - minVal;         
}

void calibrateSensor (float& ambientDC,float& dcThreshold,float& acThreshold){
  const unsigned short CAL_SAMPLES = 200;
  long sum = 0;
  for (unsigned short i = 0; i < CAL_SAMPLES; i++) {
    long v = readIR();
    sum += v;
    delay(SAMPLE_MS);
  }
  ambientDC = (float)sum / CAL_SAMPLES;
  dcThreshold = ambientDC + max(ambientDC * 0.25f, 3000.0f);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing MAX30102 ...");

  if (!synapsia.begin(Wire, I2C_SPEED_STANDARD)) { 
    Serial.println("MAX30102 not found! Please check wiring/power.");
    while(1);
  }

  // Configure sensor
  synapsia.setup();
  synapsia.setPulseAmplitudeRed(0x0A);  // low brightness for red
  synapsia.setPulseAmplitudeIR(0x0A);   // low brightness for IR

  // Calibration
  Serial.println("Calibrating: keep sensor uncovered...");
  calibrateSensor(ambientDC, dcThreshold, acThreshold);
  Serial.print("ambientDC = "); Serial.println(ambientDC);
  Serial.print("dcThreshold = "); Serial.println(dcThreshold);
  Serial.print("acThreshold = "); Serial.println(acThreshold);
  Serial.println("Ready for detection!");
}

void loop() {
  long ir = readIR();
  // push into circular window
  irWindow[index] = (float)ir;
  index++;
  if (index >= WINDOW_SIZE) { 
    index = 0;
    windowArrayFilled = true; 
  }

  float mean, p2p;
  computeWindowStats(mean, p2p);

  // detection condition: DC above threshold and AC signal present
  bool condition = (mean > dcThreshold) && (p2p > acThreshold);

  if (condition) {
    aboveCount++;
    belowCount = 0;
  } else {
    belowCount++;
    aboveCount = 0;
  }

  if (!fingerPresent && aboveCount >= DEBOUNCE_WINDOWS) {
    fingerPresent = true;
    Serial.println("Finger DETECTED");
  }
  if (fingerPresent && belowCount >= DEBOUNCE_WINDOWS) {
    fingerPresent = false;
    Serial.println("Finger REMOVED");
  }

  if(fingerPresent){
    readBpmSp02();
  }

  delay(SAMPLE_MS);
}
