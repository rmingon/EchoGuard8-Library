// SPDX-License-Identifier: CERN-OHL-S-2.0
//
// EchoGuard8: Arduino driver for the EchoGuard-8 GNSS fusion array.
// https://github.com/rmingon/EchoGuard-8
//
// The board fuses 8 GNSS receivers and exposes one solution as an SPI slave. Each time the
// chip select line goes low, it streams one 32-byte little-endian packet protected by a
// CRC-16. This library reads that packet, validates it and decodes it.

#ifndef ECHOGUARD8_H
#define ECHOGUARD8_H

#include <Arduino.h>
#include <SPI.h>

#define ECHOGUARD8_VERSION "0.1.0"

/// Confidence level computed by the board for the fused solution.
enum class EchoGuard8Status : uint8_t {
  NoFix = 0,        ///< No usable receiver.
  Ok = 1,           ///< At least 4 receivers agree, low spread, good HDOP.
  Degraded = 2,     ///< A solution exists, with reduced confidence.
  Interference = 3  ///< Receivers disagree or geometry is poor: do not rely on GNSS alone.
};

/// Reason why the last read() failed.
enum class EchoGuard8Error : uint8_t {
  None = 0,
  NoResponse,  ///< Only 0x00 or 0xFF received: board unpowered, MISO or chip select not wired.
  BadMagic,    ///< Data received but not an EchoGuard-8 packet: wrong SPI mode, speed or timing.
  BadCrc       ///< Packet corrupted in transit: lower the clock or shorten the wires.
};

/// One fused GNSS solution.
struct EchoGuard8Fix {
  bool hasFix;              ///< True when the position fields are valid.
  EchoGuard8Status status;  ///< Confidence level of this solution.

  int32_t latitudeE7;   ///< Latitude in degrees x 1e7 (exact, no rounding).
  int32_t longitudeE7;  ///< Longitude in degrees x 1e7 (exact, no rounding).
  int32_t altitudeCm;   ///< Altitude above mean sea level, in centimetres.

  uint16_t hdopCenti;      ///< Average HDOP of the receivers used, x 100.
  uint16_t maxResidualCm;  ///< Largest distance between a used receiver and the consensus, in cm.

  uint8_t usedModules;      ///< Receivers combined in this solution (0..8).
  uint8_t rejectedModules;  ///< Receivers rejected as outliers (0..8).

  uint32_t boardTickMs;  ///< Board uptime, in ms, when this solution was computed.

  /// Latitude in degrees. On 8-bit AVR boards a double has only ~7 significant digits
  /// (about 1 m): use latitudeE7 or EchoGuard8::printDegrees() when you need full precision.
  double latitude() const { return latitudeE7 / 1e7; }
  /// Longitude in degrees. Same precision note as latitude().
  double longitude() const { return longitudeE7 / 1e7; }
  /// Altitude in metres.
  float altitude() const { return altitudeCm / 100.0f; }
  /// Average horizontal dilution of precision.
  float hdop() const { return hdopCenti / 100.0f; }
  /// Spread between the receivers used, in metres.
  float maxResidual() const { return maxResidualCm / 100.0f; }
  /// True when the position is valid and the board rates it as fully trusted.
  bool isTrusted() const { return hasFix && status == EchoGuard8Status::Ok; }
};

class EchoGuard8 {
 public:
  static const uint8_t PACKET_SIZE = 32;
  static const uint8_t MODULE_COUNT = 8;
  static const uint32_t DEFAULT_CLOCK_HZ = 100000UL;
  static const uint16_t DEFAULT_SELECT_DELAY_US = 500;

  /// @param csPin  GPIO wired to the SS pin of the board (J11 pin 1).
  /// @param spi    SPI bus to use, for boards with several buses.
  explicit EchoGuard8(uint8_t csPin, SPIClass &spi = SPI);

  /// Configure the chip select pin and start the SPI bus.
  /// @param clockHz  SPI clock. The board answers from an interrupt handler: stay at or below
  ///                 the default unless you have validated a higher speed on your setup.
  void begin(uint32_t clockHz = DEFAULT_CLOCK_HZ);

  /// Time between chip select going low and the first clock edge. The board builds the
  /// packet during this window.
  void setSelectDelay(uint16_t microseconds) { _selectDelayUs = microseconds; }
  /// Extra pause between bytes, for long wires or very fast hosts. 0 by default.
  void setByteDelay(uint16_t microseconds) { _byteDelayUs = microseconds; }

  /// Read and validate one packet.
  /// @return true when a valid packet was decoded into `fix`. On failure `fix` is left
  ///         untouched and lastError() gives the reason.
  bool read(EchoGuard8Fix &fix);

  /// Read one packet without validating it. `buffer` must hold PACKET_SIZE bytes.
  void readRaw(uint8_t *buffer);

  /// Reason of the last read() failure, EchoGuard8Error::None after a success.
  EchoGuard8Error lastError() const { return _lastError; }

  /// True when the last successful read() returned a solution computed after the previous one.
  /// The board refreshes its solution 5 times per second.
  bool isUpdated() const { return _updated; }

  /// Milliseconds since the last valid packet, or 0xFFFFFFFF when none was received yet.
  uint32_t ageMs() const;

  /// Number of valid packets received since begin().
  uint32_t packetCount() const { return _packetCount; }
  /// Number of failed reads since begin().
  uint32_t errorCount() const { return _errorCount; }

  /// Validate and decode a raw 32-byte packet. Usable without any hardware.
  /// @param error  Optional, receives the failure reason.
  static bool decode(const uint8_t *packet, EchoGuard8Fix &fix, EchoGuard8Error *error = nullptr);

  /// CRC-16/CCITT-FALSE (polynomial 0x1021, initial value 0xFFFF), as used by the board.
  static uint16_t crc16(const uint8_t *data, size_t length);

  /// Human readable name: "NO_FIX", "OK", "DEGRADED", "INTERFERENCE".
  static const char *statusName(EchoGuard8Status status);
  /// Human readable name: "none", "no response", "bad magic", "bad CRC".
  static const char *errorName(EchoGuard8Error error);

  /// Ground distance in metres between two positions given in degrees x 1e7.
  /// Flat-earth approximation, accurate to better than 0.5 % below 100 km.
  static float distanceMeters(int32_t latitudeE7A, int32_t longitudeE7A, int32_t latitudeE7B,
                              int32_t longitudeE7B);

  /// Print a coordinate given in degrees x 1e7 with all 7 decimals, without any float
  /// rounding. Example: 488583012 prints "48.8583012".
  static void printDegrees(Print &out, int32_t valueE7);

 private:
  SPIClass *_spi;
  uint8_t _csPin;
  uint32_t _clockHz;
  uint16_t _selectDelayUs;
  uint16_t _byteDelayUs;

  EchoGuard8Error _lastError;
  bool _updated;
  bool _hasPacket;
  uint32_t _lastBoardTickMs;
  uint32_t _lastPacketMillis;
  uint32_t _packetCount;
  uint32_t _errorCount;
};

#endif  // ECHOGUARD8_H
