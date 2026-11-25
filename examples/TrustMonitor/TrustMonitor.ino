/*
  EchoGuard8: TrustMonitor

  Shows how to use the trust information that a single GNSS receiver cannot give you.

  The sketch polls the board 5 times per second, logs every change of status, and drives
  the built-in LED:

    LED on           solution trusted (status OK)
    slow blink       solution available with reduced confidence (DEGRADED)
    fast blink       INTERFERENCE, no fix, or board not answering

  Use the same logic in your own project to decide when to rely on GNSS and when to fall
  back on another sensor (IMU, dead reckoning, last known position).

  Wiring: see the BasicRead example.

  https://github.com/rmingon/EchoGuard-8
*/

#include <EchoGuard8.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

const uint8_t CS_PIN = 10;  // Uno / Nano: 10. ESP32: 5. Any free GPIO works.
const uint32_t POLL_INTERVAL_MS = 200;    // The board refreshes its solution at 5 Hz.
const uint32_t LINK_TIMEOUT_MS = 2000;    // No valid packet for this long: link lost.
const uint32_t SUMMARY_INTERVAL_MS = 10000;

EchoGuard8 gnss(CS_PIN);

enum Health { HEALTH_TRUSTED, HEALTH_DEGRADED, HEALTH_UNSAFE };

EchoGuard8Fix fix;
bool haveFix = false;        // At least one valid packet has been decoded.
bool linkUp = false;
EchoGuard8Status lastStatus = EchoGuard8Status::NoFix;
uint8_t lastRejected = 0;

uint32_t lastPollMs = 0;
uint32_t lastSummaryMs = 0;

Health currentHealth() {
  if (!linkUp || !fix.hasFix) {
    return HEALTH_UNSAFE;
  }
  if (fix.status == EchoGuard8Status::Ok) {
    return HEALTH_TRUSTED;
  }
  if (fix.status == EchoGuard8Status::Degraded) {
    return HEALTH_DEGRADED;
  }
  return HEALTH_UNSAFE;
}

void updateLed() {
  uint32_t now = millis();
  switch (currentHealth()) {
    case HEALTH_TRUSTED:
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case HEALTH_DEGRADED:
      digitalWrite(LED_BUILTIN, (now / 500) % 2);
      break;
    case HEALTH_UNSAFE:
      digitalWrite(LED_BUILTIN, (now / 100) % 2);
      break;
  }
}

void printSummary() {
  Serial.print(F("[summary] status "));
  Serial.print(EchoGuard8::statusName(fix.status));
  Serial.print(F("  used "));
  Serial.print(fix.usedModules);
  Serial.print(F("  rejected "));
  Serial.print(fix.rejectedModules);
  Serial.print(F("  spread "));
  Serial.print(fix.maxResidual(), 1);
  Serial.print(F(" m  HDOP "));
  Serial.print(fix.hdop(), 2);
  Serial.print(F("  packets "));
  Serial.print(gnss.packetCount());
  Serial.print(F("  errors "));
  Serial.println(gnss.errorCount());
}

void poll() {
  bool ok = gnss.read(fix);
  if (ok) {
    haveFix = true;
  }

  // Link supervision: a few failed reads are tolerated, a long silence is not.
  bool nowUp = gnss.ageMs() < LINK_TIMEOUT_MS;
  if (nowUp != linkUp) {
    linkUp = nowUp;
    if (linkUp) {
      Serial.println(F("[link] board connected"));
    } else {
      Serial.print(F("[link] LOST, last error: "));
      Serial.println(EchoGuard8::errorName(gnss.lastError()));
    }
  }
  if (!ok) {
    return;
  }

  if (fix.status != lastStatus) {
    Serial.print(F("[status] "));
    Serial.print(EchoGuard8::statusName(lastStatus));
    Serial.print(F(" -> "));
    Serial.println(EchoGuard8::statusName(fix.status));
    lastStatus = fix.status;
  }

  if (fix.rejectedModules != lastRejected) {
    Serial.print(F("[receivers] "));
    Serial.print(fix.rejectedModules);
    Serial.println(F(" rejected as outliers"));
    lastRejected = fix.rejectedModules;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
  }
  Serial.println(F("EchoGuard8 TrustMonitor"));

  gnss.begin();
}

void loop() {
  uint32_t now = millis();

  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;
    poll();
  }

  if (now - lastSummaryMs >= SUMMARY_INTERVAL_MS) {
    lastSummaryMs = now;
    if (haveFix && linkUp) {
      printSummary();
    } else {
      Serial.print(F("[link] no valid packet, last error: "));
      Serial.println(EchoGuard8::errorName(gnss.lastError()));
    }
  }

  updateLed();
}
