#include "rockert_types.h"
#include <SoftwareSerial.h>

SoftwareSerial serial_device(2, 3);

void setup() {
  /* Open serial communications and wait for port to open. */
  Serial.begin(11500);

  /* Do nonthing while the serial is not ready.*/
  while (!Serial) {
    ;
  }

  /* Set the data rate for the SoftwareSerial port */
  serial_device.begin(11500);
}

void loop() {
  message_t message_type = accel_x;
  uint_8_t data = 0;

  /* This is writing for pings two and three. */
  if (serial_device.available()) {
    serial_device.read();
    Serial.write(accel_x);
    Serial.write(data);
  }

  /* Writes on pins zero and one for usb. */
  if (Serial.available()) {
    Serial.read();
    Serial.write(accel_x);
    Serial.write(data);
  }
}