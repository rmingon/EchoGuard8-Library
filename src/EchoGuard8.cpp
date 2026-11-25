// SPDX-License-Identifier: CERN-OHL-S-2.0
//
// EchoGuard8: Arduino driver for the EchoGuard-8 GNSS fusion array.
// https://github.com/rmingon/EchoGuard-8

#include "EchoGuard8.h"

#include <math.h>

namespace {

// Packet layout (little-endian), see the "SPI interface" section of the project README.
const uint32_t kMagic = 0x31464745UL;  // "EGF1"
const uint8_t kOffsetMagic = 0;
const uint8_t kOffsetTick = 4;
const uint8_t kOffsetLatitude = 8;
const uint8_t kOffsetLongitude = 12;
const uint8_t kOffsetAltitude = 16;
const uint8_t kOffsetHdop = 20;
const uint8_t kOffsetResidual = 22;
const uint8_t kOffsetStatus = 24;
const uint8_t kOffsetUsed = 25;
const uint8_t kOffsetRejected = 26;
const uint8_t kOffsetHasFix = 27;
const uint8_t kOffsetCrc = 30;

const float kMetersPerDegree = 111320.0f;
const float kDegToRad = 0.01745329252f;

uint16_t readU16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t readU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int32_t readI32(const uint8_t *p) {
  return (int32_t)readU32(p);
}

}  // namespace

// Out-of-class definitions, needed before C++17 when a constant is passed by reference.
const uint8_t EchoGuard8::PACKET_SIZE;
const uint8_t EchoGuard8::MODULE_COUNT;
const uint32_t EchoGuard8::DEFAULT_CLOCK_HZ;
const uint16_t EchoGuard8::DEFAULT_SELECT_DELAY_US;

EchoGuard8::EchoGuard8(uint8_t csPin, SPIClass &spi)
    : _spi(&spi),
      _csPin(csPin),
      _clockHz(DEFAULT_CLOCK_HZ),
      _selectDelayUs(DEFAULT_SELECT_DELAY_US),
      _byteDelayUs(0),
      _lastError(EchoGuard8Error::None),
      _updated(false),
      _hasPacket(false),
      _lastBoardTickMs(0),
      _lastPacketMillis(0),
      _packetCount(0),
      _errorCount(0) {}

void EchoGuard8::begin(uint32_t clockHz) {
  _clockHz = clockHz;
  digitalWrite(_csPin, HIGH);  // Set the level first so the line never glitches low.
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin();

  _lastError = EchoGuard8Error::None;
  _updated = false;
  _hasPacket = false;
  _packetCount = 0;
  _errorCount = 0;
}

void EchoGuard8::readRaw(uint8_t *buffer) {
  _spi->beginTransaction(SPISettings(_clockHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  // The board builds the packet and loads its first byte when chip select falls.
  delayMicroseconds(_selectDelayUs);
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    buffer[i] = _spi->transfer(0x00);
    if (_byteDelayUs != 0) {
      delayMicroseconds(_byteDelayUs);
    }
  }
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
}

bool EchoGuard8::read(EchoGuard8Fix &fix) {
  uint8_t packet[PACKET_SIZE];
  readRaw(packet);

  EchoGuard8Fix decoded;
  if (!decode(packet, decoded, &_lastError)) {
    _errorCount++;
    _updated = false;
    return false;
  }

  _updated = !_hasPacket || (decoded.boardTickMs != _lastBoardTickMs);
  _hasPacket = true;
  _lastBoardTickMs = decoded.boardTickMs;
  _lastPacketMillis = millis();
  _packetCount++;

  fix = decoded;
  return true;
}

uint32_t EchoGuard8::ageMs() const {
  if (!_hasPacket) {
    return 0xFFFFFFFFUL;
  }
  return millis() - _lastPacketMillis;
}

bool EchoGuard8::decode(const uint8_t *packet, EchoGuard8Fix &fix, EchoGuard8Error *error) {
  EchoGuard8Error result = EchoGuard8Error::None;

  if (readU32(packet + kOffsetMagic) != kMagic) {
    // A floating or stuck MISO line reads as all zeros or all ones.
    bool allSame = true;
    for (uint8_t i = 1; i < PACKET_SIZE; i++) {
      if (packet[i] != packet[0]) {
        allSame = false;
        break;
      }
    }
    bool idleLine = allSame && (packet[0] == 0x00 || packet[0] == 0xFF);
    result = idleLine ? EchoGuard8Error::NoResponse : EchoGuard8Error::BadMagic;
  } else if (crc16(packet, kOffsetCrc) != readU16(packet + kOffsetCrc)) {
    result = EchoGuard8Error::BadCrc;
  }

  if (error != nullptr) {
    *error = result;
  }
  if (result != EchoGuard8Error::None) {
    return false;
  }

  fix.boardTickMs = readU32(packet + kOffsetTick);
  fix.latitudeE7 = readI32(packet + kOffsetLatitude);
  fix.longitudeE7 = readI32(packet + kOffsetLongitude);
  fix.altitudeCm = readI32(packet + kOffsetAltitude);
  fix.hdopCenti = readU16(packet + kOffsetHdop);
  fix.maxResidualCm = readU16(packet + kOffsetResidual);
  uint8_t status = packet[kOffsetStatus];
  // A status added by a newer firmware is reported as Degraded rather than trusted.
  fix.status = (status <= (uint8_t)EchoGuard8Status::Interference) ? (EchoGuard8Status)status
                                                                    : EchoGuard8Status::Degraded;
  fix.usedModules = packet[kOffsetUsed];
  fix.rejectedModules = packet[kOffsetRejected];
  fix.hasFix = packet[kOffsetHasFix] != 0;
  return true;
}

uint16_t EchoGuard8::crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc & 0x8000) != 0) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

const char *EchoGuard8::statusName(EchoGuard8Status status) {
  switch (status) {
    case EchoGuard8Status::NoFix:
      return "NO_FIX";
    case EchoGuard8Status::Ok:
      return "OK";
    case EchoGuard8Status::Degraded:
      return "DEGRADED";
    case EchoGuard8Status::Interference:
      return "INTERFERENCE";
  }
  return "?";
}

const char *EchoGuard8::errorName(EchoGuard8Error error) {
  switch (error) {
    case EchoGuard8Error::None:
      return "none";
    case EchoGuard8Error::NoResponse:
      return "no response";
    case EchoGuard8Error::BadMagic:
      return "bad magic";
    case EchoGuard8Error::BadCrc:
      return "bad CRC";
  }
  return "?";
}

float EchoGuard8::distanceMeters(int32_t latitudeE7A, int32_t longitudeE7A, int32_t latitudeE7B,
                                 int32_t longitudeE7B) {
  // Work on integer differences: they stay exact, where a float degree value would already
  // be rounded to about a metre on boards with 32-bit floats.
  int64_t dLatE7 = (int64_t)latitudeE7B - (int64_t)latitudeE7A;
  int64_t dLonE7 = (int64_t)longitudeE7B - (int64_t)longitudeE7A;
  if (dLonE7 > 1800000000LL) {  // Shortest way across the antimeridian.
    dLonE7 -= 3600000000LL;
  } else if (dLonE7 < -1800000000LL) {
    dLonE7 += 3600000000LL;
  }

  float meanLatitudeDeg = ((float)latitudeE7A * 0.5f + (float)latitudeE7B * 0.5f) * 1e-7f;
  float north = (float)dLatE7 * 1e-7f * kMetersPerDegree;
  float east = (float)dLonE7 * 1e-7f * kMetersPerDegree * cosf(meanLatitudeDeg * kDegToRad);
  return sqrtf(north * north + east * east);
}

void EchoGuard8::printDegrees(Print &out, int32_t valueE7) {
  uint32_t magnitude = (valueE7 < 0) ? (uint32_t)(-(int64_t)valueE7) : (uint32_t)valueE7;
  if (valueE7 < 0) {
    out.print('-');
  }
  out.print(magnitude / 10000000UL);
  out.print('.');
  uint32_t fraction = magnitude % 10000000UL;
  for (uint32_t divisor = 1000000UL; divisor > 0; divisor /= 10) {
    out.print((char)('0' + (fraction / divisor) % 10));
  }
}
