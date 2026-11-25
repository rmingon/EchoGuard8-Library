/*
  EchoGuard8: BasicRead

  Reads the fused position of an EchoGuard-8 GNSS array once per second and prints it.

  Wiring (connector J11 of the board, 3.3 V logic):

    J11 pin 1  SS     ->  CS_PIN (any free GPIO)
    J11 pin 2  MOSI   ->  MOSI
    J11 pin 3  MISO   ->  MISO
    J11 pin 4  SCK    ->  SCK
    J11 pin 5  GND    ->  GND

  The board uses 3.3 V logic: with a 5 V Arduino (Uno, Nano, Mega), use a level shifter.
  Power the board from its own 3.3 V supply, not from the 3.3 V pin of an Uno.

  https://github.com/rmingon/EchoGuard-8
*/

#include <EchoGuard8.h>

const uint8_t CS_PIN = 10;  // Uno / Nano: 10. ESP32: 5. Any free GPIO works.

EchoGuard8 gnss(CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for the USB serial port on boards that have one.
  }
  Serial.println(F("EchoGuard8 BasicRead"));

  gnss.begin();
}

void loop() {
  EchoGuard8Fix fix;

  if (!gnss.read(fix)) {
    Serial.print(F("Read failed: "));
    Serial.println(EchoGuard8::errorName(gnss.lastError()));
  } else if (!fix.hasFix) {
    Serial.println(F("Waiting for a fix..."));
  } else {
    Serial.print(F("lat "));
    EchoGuard8::printDegrees(Serial, fix.latitudeE7);
    Serial.print(F("  lon "));
    EchoGuard8::printDegrees(Serial, fix.longitudeE7);
    Serial.print(F("  alt "));
    Serial.print(fix.altitude(), 1);
    Serial.print(F(" m  status "));
    Serial.print(EchoGuard8::statusName(fix.status));
    Serial.print(F("  receivers "));
    Serial.print(fix.usedModules);
    Serial.print('/');
    Serial.println(EchoGuard8::MODULE_COUNT);
  }

  delay(1000);
}
