#include <Wire.h>
#include "MAX30105.h"

#define ledBrightness 0x1F
#define sampleAverge  4
#define ledMode 2
#define sampleRate 400
#define pulseWidth 411
#define ADCRange 4096


MAX30105 wristSensor;

void setup() {
  Serial.begin(115200);
  Serial.println("MAX30102 Wrist Test");

  if (!wristSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Please check wiring/power.");
    while (1);
  }

  wristSensor.setup(ledBrightness, sampleAverge, ledMode, sampleRate, pulseWidth, ADCRange);
  
  Serial.print("Reading sesnor data");
  
  unsigned short int i = 0;
  while(i<4){
    Serial.print("."); 
    delay(300);
    i++;
  }

  Serial.println();
}

void loop() {

  unsigned long irValue = wristSensor.getIR();
  unsigned long redValue = wristSensor.getRed();

  Serial.print("IR = ");
  Serial.print(irValue);
  Serial.print(", Red = ");
  Serial.println(redValue);

  delay(200);
}
