/* Test sketch for Adafruit BNO08x sensor in UART-RVC mode */

#include "Adafruit_BNO08x_RVC.h"

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

void setup() {
  // Wait for serial monitor to open
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit BNO08x IMU - UART-RVC mode");

  Serial1.begin(115200); // This is the baud rate specified by the datasheet
  while (!Serial1)
    delay(10);

  if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
    Serial.println("Could not find BNO08x!");
    while (1)
      delay(10);
  }

  Serial.println("BNO08x found!");
}

void loop() {
  BNO08x_RVC_Data heading;

  if (!rvc.read(&heading)) {
    return;
  }

  Serial.println();
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Principal Axes:"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("Yaw: "));
  Serial.print(heading.yaw);
  Serial.print(F("\tPitch: "));
  Serial.print(heading.pitch);
  Serial.print(F("\tRoll: "));
  Serial.println(heading.roll);
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Acceleration"));
  Serial.println(F("---------------------------------------"));
  Serial.print(F("X: "));
  Serial.print(heading.x_accel);
  Serial.print(F("\tY: "));
  Serial.print(heading.y_accel);
  Serial.print(F("\tZ: "));
  Serial.println(heading.z_accel);
  Serial.println(F("---------------------------------------"));


  //  delay(200);
}
