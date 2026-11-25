# EchoGuard8 for Arduino

Read the fused position of an [EchoGuard-8](https://github.com/rmingon/EchoGuard-8) GNSS
array from any Arduino board, in three lines of code.

EchoGuard-8 is an open hardware board that runs 8 GNSS receivers side by side, cross-checks
them, rejects the ones that disagree and outputs **one position with a trust status**. This
library reads that solution over SPI.

```cpp
#include <EchoGuard8.h>

EchoGuard8 gnss(10);  // chip select pin

void setup() {
  Serial.begin(115200);
  gnss.begin();
}

void loop() {
  EchoGuard8Fix fix;
  if (gnss.read(fix) && fix.hasFix) {
    EchoGuard8::printDegrees(Serial, fix.latitudeE7);
    Serial.print(", ");
    EchoGuard8::printDegrees(Serial, fix.longitudeE7);
    Serial.print("  ");
    Serial.println(EchoGuard8::statusName(fix.status));
  }
  delay(1000);
}
```

```text
48.8583012, 2.2945004  OK
```

## Features

- **One call** to get latitude, longitude, altitude and HDOP
- **Trust status** on every solution: `OK`, `DEGRADED`, `INTERFERENCE`, `NO_FIX`
- **Array health**: receivers used, receivers rejected, spread between receivers
- **Every packet is CRC-checked**; a corrupted read is refused, never returned
- **Clear diagnostics** when something is wrong: no response, bad magic, bad CRC
- **Full precision on 8-bit boards**: coordinates stay in integer degrees × 10⁷
- Helpers for distance, coordinate printing and link supervision
- No dependency beyond the `SPI` library shipped with every Arduino core

## Wiring

The board exposes its SPI port on connector `J11` (JST-SH, 6 pins).

| J11 pin | Signal | Connect to |
| :---: | --- | --- |
| 1 | `SS` (active low) | Any free GPIO, the `csPin` given to the constructor |
| 2 | `MOSI` | SPI `MOSI` |
| 3 | `MISO` | SPI `MISO` |
| 4 | `SCK` | SPI `SCK` |
| 5 | `GND` | `GND` |
| 6 | `+3.3V` | The 3.3 V rail of the board, see below |

Two points to get right:

- **3.3 V logic.** With a 5 V Arduino (Uno, Nano, Mega), put a level shifter on the SPI
  lines. Boards that already run at 3.3 V (ESP32, SAMD, RP2040, STM32, Nano 33) connect
  directly.
- **Power.** Eight receivers draw far more than the 3.3 V pin of an Uno can supply. Power
  the board from its own regulated 3.3 V supply and budget about 300 mA, more with active
  antennas. Always connect the grounds together.

## Installation

**Arduino IDE:** open *Sketch > Include Library > Manage Libraries*, search for
`EchoGuard8` and click *Install*.

**Arduino CLI:**

```bash
arduino-cli lib install EchoGuard8
```

**Manual:** download this repository as a ZIP, then use
*Sketch > Include Library > Add .ZIP Library*.

## Examples

Open them from *File > Examples > EchoGuard8*.

| Example | What it shows |
| --- | --- |
| **BasicRead** | The minimum: read and print the position once per second. Start here. |
| **TrustMonitor** | Using the trust status: logs every status change, supervises the link, and drives an LED (on, slow blink, fast blink). |
| **Geofence** | A complete application: records a home position, measures the distance to it and raises an alarm outside a radius, using trusted solutions only. |
| **PacketDump** | Wiring diagnostic: prints the raw packets in hexadecimal and lets you change the SPI clock from the serial monitor. |

## API

### `EchoGuard8`

| Method | Description |
| --- | --- |
| `EchoGuard8(csPin, spi = SPI)` | Create the driver. Pass another `SPIClass` for boards with several buses. |
| `begin(clockHz = 100000)` | Configure the chip select pin and start the SPI bus. |
| `bool read(fix)` | Read one packet. Returns `true` when it is valid and decoded into `fix`. On failure `fix` is left untouched. |
| `lastError()` | Why the last `read()` failed: `None`, `NoResponse`, `BadMagic`, `BadCrc`. |
| `isUpdated()` | `true` when the last read returned a solution newer than the previous one. |
| `ageMs()` | Milliseconds since the last valid packet. Use it to detect a lost link. |
| `packetCount()`, `errorCount()` | Counters since `begin()`. |
| `readRaw(buffer)` | Read 32 raw bytes without validation. |
| `setSelectDelay(us)` | Delay between chip select and the first clock edge. 500 µs by default. |
| `setByteDelay(us)` | Optional pause between bytes. 0 by default. |

Static helpers, usable without a board:

| Function | Description |
| --- | --- |
| `decode(packet, fix, &error)` | Validate and decode a raw 32-byte packet. |
| `distanceMeters(latA, lonA, latB, lonB)` | Ground distance between two positions in degrees × 10⁷. |
| `printDegrees(Serial, valueE7)` | Print a coordinate with its 7 decimals, without float rounding. |
| `statusName(status)`, `errorName(error)` | Human readable names. |
| `crc16(data, length)` | The CRC-16/CCITT-FALSE used by the board. |

### `EchoGuard8Fix`

| Field | Type | Description |
| --- | --- | --- |
| `hasFix` | `bool` | The position fields are valid. |
| `status` | `EchoGuard8Status` | Trust level, see below. |
| `latitudeE7`, `longitudeE7` | `int32_t` | Degrees × 10⁷, exact. |
| `altitudeCm` | `int32_t` | Altitude in centimetres. |
| `hdopCenti` | `uint16_t` | Average HDOP × 100. |
| `maxResidualCm` | `uint16_t` | Largest distance between a used receiver and the consensus. |
| `usedModules`, `rejectedModules` | `uint8_t` | Receivers combined and receivers rejected, out of 8. |
| `boardTickMs` | `uint32_t` | Board uptime when the solution was computed. |

Convenience accessors: `latitude()`, `longitude()` (degrees), `altitude()` (metres),
`hdop()`, `maxResidual()` (metres) and `isTrusted()` (`hasFix` and status `OK`).

> On 8-bit AVR boards, `double` is a 32-bit float with about 7 significant digits, which is
> roughly 1 m on a coordinate. Use `latitudeE7` / `longitudeE7`, `printDegrees()` and
> `distanceMeters()` to keep the full resolution.

### Trust status

| Status | Meaning | Suggested use |
| --- | --- | --- |
| `Ok` | At least 4 receivers agree, small spread, good geometry | Use the position |
| `Degraded` | A solution exists with reduced confidence | Use with caution, widen your margins |
| `Interference` | Receivers disagree or geometry is poor | Do not rely on GNSS alone |
| `NoFix` | No usable receiver | No position |

## Timing

The board prepares its packet at the moment chip select goes low, and answers from an
interrupt handler. The defaults are deliberately conservative:

- **500 µs** between chip select and the first clock edge
- **100 kHz** SPI clock (a full packet takes about 3 ms)

The solution is refreshed 5 times per second, so reading faster than every 200 ms returns
the same data. If you want a faster bus, validate it on your wiring with the `PacketDump`
example: the CRC makes a marginal setup visible immediately.

## Troubleshooting

| `lastError()` | Likely cause |
| --- | --- |
| `NoResponse` | Board not powered, `MISO` or `SS` not connected, missing common ground, wrong `csPin`. |
| `BadMagic` | Data arrives but scrambled: clock too fast, select delay too short, `SCK` and `MOSI` swapped, missing level shifter. |
| `BadCrc` | Almost right: lower the clock, shorten the wires, add a byte delay. |
| Valid packets, `hasFix` stays `false` | The link works. The receivers have no fix yet: check the antennas and the sky view, a cold start takes around half a minute. |

## Compatibility

The library uses only the standard Arduino SPI API and has no architecture-specific code.
It is compiled in CI for AVR (Uno) and ESP32.

## License

Released under the same licence as the EchoGuard-8 project, the **CERN Open Hardware Licence
Version 2, Strongly Reciprocal** (CERN-OHL-S v2). See [LICENSE](./LICENSE).
