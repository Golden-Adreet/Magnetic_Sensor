#include <Wire.h>
#include <DFRobot_GP8XXX.h>

#define DAC_ADDRESS 0x59

DFRobot_GP8413 DAC(DAC_ADDRESS);

void setup() {

  Serial.begin(115200);
  Wire.begin();

  Serial.println("Starting DAC...");

  if (DAC.begin() != 0) {
    Serial.println("ERROR: DAC not found!");
    while (1);
  }

  Serial.println("DAC found!");

  // Set output range to 0–10 V
  DAC.setDACOutRange(DAC.eOutputRange10V);

  delay(1000);

  // Output approximately 5 V on channel 0
  DAC.setDACOutVoltage(16384, 0);

  Serial.println("Output set to approximately 5 V");
}

void loop() {
}
