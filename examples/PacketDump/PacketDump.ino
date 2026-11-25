/*
  EchoGuard8: PacketDump

  Bring-up and wiring diagnostic. Prints every raw 32-byte packet in hexadecimal, then
  either the decoded fields or the reason why the packet was refused.

  What the result tells you:

    no response   only 00 or FF bytes: board not powered, MISO or SS not connected,
                  missing common ground
    bad magic     data arrives but shifted or scrambled: wrong SPI mode, clock too fast,
                  select delay too short
    bad CRC       almost right: lower the clock, shorten the wires

  Type '+' or '-' in the serial monitor to change the SPI clock on the fly and find the
  highest speed that works with your wiring.

  Wiring: see the BasicRead example.

  https://github.com/rmingon/EchoGuard-8
*/

#include <EchoGuard8.h>

const uint8_t CS_PIN = 10;  // Uno / Nano: 10. ESP32: 5. Any free GPIO works.

const uint32_t CLOCKS_HZ[] = {25000UL, 50000UL, 100000UL, 250000UL, 500000UL, 1000000UL};
const uint8_t CLOCK_COUNT = sizeof(CLOCKS_HZ) / sizeof(CLOCKS_HZ[0]);
uint8_t clockIndex = 2;  // 100 kHz, the library default.

EchoGuard8 gnss(CS_PIN);

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void applyClock() {
  gnss.begin(CLOCKS_HZ[clockIndex]);
  Serial.print(F("SPI clock: "));
  Serial.print(CLOCKS_HZ[clockIndex] / 1000UL);
  Serial.println(F(" kHz"));
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '+' && clockIndex + 1 < CLOCK_COUNT) {
      clockIndex++;
      applyClock();
    } else if (c == '-' && clockIndex > 0) {
      clockIndex--;
      applyClock();
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println(F("EchoGuard8 PacketDump  ('+' / '-' change the SPI clock)"));

  applyClock();
}

void loop() {
  handleSerial();

  uint8_t packet[EchoGuard8::PACKET_SIZE];
  gnss.readRaw(packet);

  for (uint8_t i = 0; i < EchoGuard8::PACKET_SIZE; i++) {
    printHexByte(packet[i]);
    Serial.print((i % 4 == 3) ? F("  ") : F(" "));
  }
  Serial.println();

  EchoGuard8Fix fix;
  EchoGuard8Error error;
  if (!EchoGuard8::decode(packet, fix, &error)) {
    Serial.print(F("  -> refused: "));
    Serial.println(EchoGuard8::errorName(error));
  } else {
    Serial.print(F("  -> valid  tick "));
    Serial.print(fix.boardTickMs);
    Serial.print(F(" ms  status "));
    Serial.print(EchoGuard8::statusName(fix.status));
    Serial.print(F("  fix "));
    Serial.print(fix.hasFix ? F("yes") : F("no"));
    Serial.print(F("  used "));
    Serial.print(fix.usedModules);
    Serial.print(F("  rejected "));
    Serial.print(fix.rejectedModules);
    if (fix.hasFix) {
      Serial.print(F("  lat "));
      EchoGuard8::printDegrees(Serial, fix.latitudeE7);
      Serial.print(F("  lon "));
      EchoGuard8::printDegrees(Serial, fix.longitudeE7);
    }
    Serial.println();
  }

  delay(500);
}
