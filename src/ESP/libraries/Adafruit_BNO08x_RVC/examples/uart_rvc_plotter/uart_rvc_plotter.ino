/* Plotter-friendly version of the test sketch for Adafruit BNO08x sensor in
 * UART-RVC mode */
#include "Adafruit_BNO08x_RVC.h"

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

void setup() {
  // Wait for serial monitor to open
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial1.begin(115200); // This is the baud rate specified by the datasheet
  while (!Serial1)
    delay(10);

  if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
    while (1)
      delay(10);
  }
  // Print labels for the values
  Serial.print(F("Yaw"));
  Serial.print(F("\tPitch"));
  Serial.print(F("\tRoll"));
  Serial.print(F("\tX"));
  Serial.print(F("\tY"));
  Serial.println(F("\tZ"));
}

void loop() {
  BNO08x_RVC_Data heading;

  if (!rvc.read(&heading)) {
    // Data not available or parsable, keep trying
    return;
  }

  Serial.print(heading.yaw);Serial.print(F(","));
  Serial.print(heading.pitch);Serial.print(F(","));
  Serial.print(heading.roll);Serial.print(F(","));
  Serial.print(heading.x_accel);Serial.print(F(","));
  Serial.print(heading.y_accel);Serial.print(F(","));
  Serial.print(heading.z_accel);
  Serial.println("");
}
