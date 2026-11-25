/*
  EchoGuard8: Geofence

  Records a home position, then reports the distance to it and raises an alarm when the
  board leaves a circle around home.

  Only trusted solutions (status OK) are used, both to record home and to trigger the
  alarm: a degraded or disturbed position must never move the fence or raise a false alert.

  Wiring: see the BasicRead example. The alarm output is the built-in LED: replace it with
  a buzzer, a relay or a radio message.

  https://github.com/rmingon/EchoGuard-8
*/

#include <EchoGuard8.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const uint8_t CS_PIN = 10;  // Uno / Nano: 10. ESP32: 5. Any free GPIO works.
const uint8_t ALARM_PIN = LED_BUILTIN;

const float FENCE_RADIUS_M = 50.0f;
const float FENCE_HYSTERESIS_M = 5.0f;  // Avoids an alarm that flickers on the boundary.
const uint8_t HOME_SAMPLES = 10;        // Trusted solutions averaged to record home.

EchoGuard8 gnss(CS_PIN);

bool homeSet = false;
int32_t homeLatitudeE7 = 0;
int32_t homeLongitudeE7 = 0;

// Home is averaged as offsets from the first sample, which keeps the sums small and exact.
int32_t firstLatitudeE7 = 0;
int32_t firstLongitudeE7 = 0;
int32_t sumLatitudeOffset = 0;
int32_t sumLongitudeOffset = 0;
uint8_t sampleCount = 0;

bool outside = false;

void recordHome(const EchoGuard8Fix &fix) {
  if (sampleCount == 0) {
    firstLatitudeE7 = fix.latitudeE7;
    firstLongitudeE7 = fix.longitudeE7;
  }
  sumLatitudeOffset += fix.latitudeE7 - firstLatitudeE7;
  sumLongitudeOffset += fix.longitudeE7 - firstLongitudeE7;
  sampleCount++;

  Serial.print(F("Recording home "));
  Serial.print(sampleCount);
  Serial.print('/');
  Serial.println(HOME_SAMPLES);

  if (sampleCount >= HOME_SAMPLES) {
    homeLatitudeE7 = firstLatitudeE7 + sumLatitudeOffset / HOME_SAMPLES;
    homeLongitudeE7 = firstLongitudeE7 + sumLongitudeOffset / HOME_SAMPLES;
    homeSet = true;

    Serial.print(F("Home set: "));
    EchoGuard8::printDegrees(Serial, homeLatitudeE7);
    Serial.print(F(", "));
    EchoGuard8::printDegrees(Serial, homeLongitudeE7);
    Serial.print(F("  fence radius "));
    Serial.print(FENCE_RADIUS_M, 0);
    Serial.println(F(" m"));
  }
}

void checkFence(const EchoGuard8Fix &fix) {
  float distance = EchoGuard8::distanceMeters(homeLatitudeE7, homeLongitudeE7, fix.latitudeE7,
                                              fix.longitudeE7);

  if (!outside && distance > FENCE_RADIUS_M + FENCE_HYSTERESIS_M) {
    outside = true;
    Serial.println(F("ALARM: fence left"));
  } else if (outside && distance < FENCE_RADIUS_M - FENCE_HYSTERESIS_M) {
    outside = false;
    Serial.println(F("Back inside the fence"));
  }
  digitalWrite(ALARM_PIN, outside ? HIGH : LOW);

  Serial.print(F("Distance to home: "));
  Serial.print(distance, 1);
  Serial.print(F(" m  ("));
  Serial.print(fix.usedModules);
  Serial.println(F(" receivers)"));
}

void setup() {
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println(F("EchoGuard8 Geofence"));

  gnss.begin();
}

void loop() {
  EchoGuard8Fix fix;

  if (!gnss.read(fix)) {
    Serial.print(F("Read failed: "));
    Serial.println(EchoGuard8::errorName(gnss.lastError()));
  } else if (!fix.isTrusted()) {
    // Keep the last alarm state: an untrusted position proves nothing either way.
    Serial.print(F("Position not trusted ("));
    Serial.print(EchoGuard8::statusName(fix.status));
    Serial.println(F("), ignored"));
  } else if (!homeSet) {
    recordHome(fix);
  } else {
    checkFence(fix);
  }

  delay(1000);
}
