# LifeLink - Functionality Reference

## Overview

Raspberry Pi Pico-based vitals monitor. Reads heart rate and SpO2 from a
MAX30102, body temperature from a MAX30205, and streams the readings out
two ways simultaneously: human-readable lines on the USB serial port for
desktop monitoring, and the same lines over a HM-10 BLE module for any
phone/tablet running a generic BLE serial app.

The MAX30205 OS/INT pin is configured as an over-temperature alert. The
firmware has no fall detection or on-device UI.

---

## Sensors

| Sensor | Bus | Address | Provides |
|---|---|---|---|
| MAX30102 | I2C0 (SDA=GP4, SCL=GP5) | 0x57 | 18-bit Red + IR optical samples -> derived HR & SpO2 |
| MAX30205 | I2C1 (SDA=GP6, SCL=GP7) | 0x48 | 16-bit signed body temperature, 0.00390625 C / LSB |
| HM-10    | UART0 (9600 8N1) | - | Transparent BLE/UART bridge, output only; PIN-paired (see BLE Security) |

Each sensor has its own hardware I2C block and shares no SDA/SCL pins with
the other — the RP2040 has exactly two I2C controllers (i2c0, i2c1), one per
sensor. This also isolates the two buses: a device that jams SDA/SCL low can
only stall its own sensor, not both. MAX30102 INT is GP2. MAX30205 OS/INT is GP3.

### Reference hardware (verified compatible)

| Part | Board used | Notes |
|---|---|---|
| MAX30102 | generic purple GY-MAX30102 breakout | 3.3 V, onboard I2C pull-ups |
| MAX30205 | generic MAX30205 I2C breakout | 3.3 V, onboard pull-ups; `OS` pin optional |
| HM-10 | AITIAO / AT-09 (CC2541) | 3.3 V logic, 9600 baud |

All three run on the Pico's 3.3 V rail, wired in parallel to a shared
breadboard rail — no external regulator and no power resistors. See
`connections.txt` for the exact pin-by-pin wiring.

The AT-09 is an HM-10-class clone; its `AT+PASS`/`AT+TYPE` pairing dialect
can differ from a genuine HMSoft module. The boot pairing step is bounded
and self-checking (see BLE Security), so on an incompatible clone it simply
reports `not configured` and the vitals stream still works unencrypted.

---

## Output Format

One line per publish interval, identical text on USB serial and HM-10:

```
HR=72 SpO2=98 BodyT=36.50C
HR=-- SpO2=-- BodyT=36.48C  [no finger]
HR=-- SpO2=-- BodyT=36.51C  [MAX30102 offline]
HR=72 SpO2=98 BodyT=--      [MAX30205 offline]
HR=-- SpO2=-- BodyT=--      [no sensors]
```

`HR` is in beats per minute, `SpO2` is a percentage, `BodyT` is degrees
Celsius. Any value the firmware cannot measure is rendered as `--`,
and the trailing tag tells the consumer why, distinguishing
"finger off the sensor" from "sensor unplugged" from "everything is
gone".

---

## HR / SpO2 Algorithm

The MAX30102 is configured for SpO2 mode:

| Setting | Value |
|---|---|
| Sample rate | 100 Hz raw |
| Sample averaging (`SMP_AVE`) | 4 -> FIFO output about 25 Hz |
| ADC range | 4096 nA |
| LED pulse width | 411 µs (18-bit) |
| Red / IR LED current | ~7 mA each (0x24) |

The MAX30102 INT pin wakes the Pico when FIFO data is available. The
firmware drains the on-chip 32-entry FIFO into a sliding 100-sample
ring buffer, about 4 seconds at 25 Hz output rate. Once the window is
full, the oldest sample is overwritten in place (O(1) per sample, no
shifting), so HR/SpO2 is always computed over the most recent four
seconds of data.  At each publish tick:

1. **Finger detection** - IR DC level must exceed `FINGER_DC_THRESH`
   (50 000) or the reading is suppressed.
2. **Heart rate** - local-maximum peak detection on the IR channel,
   threshold = mean + 25 % of peak-to-peak amplitude, minimum spacing
   400 ms (about 150 bpm cap). Result = `60 * SAMPLE_RATE_HZ / mean_peak_interval`,
   averaging the sample spacing between consecutive detected peaks
   rather than dividing the peak count by the window length (peak
   count over a fixed 4 s window would otherwise quantise bpm to
   multiples of 15). Values outside 30–220 bpm are rejected.
3. **SpO2** - ratio-of-ratios:
   `R = (AC_red/DC_red) / (AC_ir/DC_ir)` then
   `SpO2 ~= 110 - 25 * R`, clamped to 70–100 %.

These are textbook approximations and are not clinically calibrated.

---

## Periodic Tasks

| Task | Cadence |
|---|---|
| Drain MAX30102 FIFO into sample window | MAX30102 INT, plus 500 ms backstop |
| Compute HR + SpO2, read MAX30205, publish line | Every 2 s (`READ_INTERVAL_MS`) |
| Idle wake for watchdog and timers | Every 100 ms (`IDLE_WAKE_INTERVAL_MS`) |

---

## Boot Sequence

1. `stdio_init_all()` - bring up USB CDC, then wait up to 5 s for a host
   to attach so the boot banner isn't lost.
2. Initialise both I2C buses @ 400 kHz with internal pull-ups on each
   SDA/SCL (I2C0 GP4/GP5 for the MAX30102, I2C1 GP6/GP7 for the MAX30205;
   use external 4.7 kΩ pull-ups in addition for reliable fast-mode).
3. Initialise UART0 @ 9600 8N1 for the HM-10.
4. Configure HM-10 BLE pairing (once, before any central connects): send
   `AT`, log the `AT+VERS?` response for module confirmation, then set the
   PIN (`AT+PASS`) and require auth + bonding (`AT+TYPE2`), and `AT+RESET`.
   Bounded and non-blocking — an absent or already-connected module is
   reported and boot continues. See "BLE Security" below.
5. Configure GP2 and GP3 as active-low interrupt inputs.
6. Probe and configure the MAX30102 (PART_ID check, soft reset,
   FIFO + SpO2 config, LED currents, FIFO interrupts).
7. Probe the MAX30205, switch it to continuous-conversion mode, and set
   the OS/INT alert thresholds.
8. Print the status banner and arm the watchdog (8 s timeout).
9. Enter the publish loop.

A failed sensor probe is reported on the USB serial banner and the loop
continues. The MAX30205 read is independent of the MAX30102, so a
missing optical sensor still leaves body temperature working, and vice
versa.

If the USB host attaches *after* boot, the firmware re-emits the banner
on the next loop iteration so a late attacher always sees current
sensor status.

---

## BLE Security

By default an HM-10 accepts open, unauthenticated connections
(`AT+TYPE0`, no PIN), so any nearby BLE central could read the live
vitals stream. At boot, before any phone connects, the firmware
configures the module for authenticated + bonded connections and a
numeric pairing PIN:

| Command | Purpose |
|---|---|
| `AT` | Handshake gate — if there is no `OK`, nothing else is sent |
| `AT+VERS?` | Logged to USB so a human can confirm it is a genuine HM-10 |
| `AT+PASS<pin>` | Sets the 6-digit pairing PIN (`HM10_PAIR_PIN`) |
| `AT+TYPE2` | Requires authentication + bonding (replaces open `AT+TYPE0`) |
| `AT+RESET` | Latches the new settings (some firmware applies them only on restart) |

The PIN is injected at build time so a real PIN never lives in tracked
source; `config.h` only holds an obvious `"000000"` placeholder behind an
`#ifndef` guard:

```bash
cmake -DHM10_PAIR_PIN=481920 ..   # forwarded as -DHM10_PAIR_PIN="481920"
```

```c
/* config.h fallback if the build-time value is not supplied */
#ifndef HM10_PAIR_PIN
#define HM10_PAIR_PIN "000000"   /* placeholder - override at build time */
#endif
```

The placeholder **must** be overridden. A
phone/tablet must enter this PIN the first time it connects (in
nRF Connect, Serial Bluetooth Terminal, etc.) before the stream
becomes readable.

Properties of this step:

- Runs **exactly once**, at boot, from `lifelink_init()` right after
  `hm10_init()` — never from the main loop, since re-issuing
  `AT+PASS`/`AT+TYPE` to a connected module can drop the client.
- AT commands only work while the module is **not** connected to a
  central, which is why it runs in the boot window.
- Every read/write is bounded by a timeout, so an unplugged,
  unresponsive, or already-connected module never blocks boot. It runs
  before the watchdog is armed, so it cannot trip a watchdog reset.
- The banner reports `HM-10 pairing: OK (PIN required)` on success or
  `not configured` on failure (module left with its previous security).

Note: "HM-10" is used loosely for several chipsets (genuine
HM-10/HMSoft, AT-09, JDY-08, HC-08, …) whose AT dialects differ.
Confirm the logged `AT+VERS?` string is a genuine HM-10 before trusting
this sequence on the wearable's module — a wrong `AT+TYPE`/`AT+PASS` on
some clones can require an `AT+RENEW` factory reset to recover. Test on
a spare module first.

---

## Crash & Unplug Safety

| Failure mode | Behaviour |
|---|---|
| Sensor unplugged at boot | Banner reports `OFFLINE`; loop publishes `--` for that sensor |
| Sensor unplugged mid-run (MAX30102) | 3 s heartbeat: no samples, marked OFFLINE, status tag updated |
| Sensor unplugged mid-run (MAX30205) | Failed I2C ACK, marked OFFLINE on the next read attempt |
| Sensor re-plugged mid-run | Reprobed every 5 s; logs `MAX30102 came online` / `MAX30205 came online` |
| I2C bus stuck low | All transactions time out at 10 ms; loop continues |
| HM-10 unplugged | Writes use a bounded UART timeout; unread data is dropped rather than stalling |
| USB host disconnects | tinyusb drops bytes; no blocking, no crash |
| Loop wedged for any reason | Hardware watchdog reboots the Pico after 8 s; the new banner notes the watchdog reset |

---

## Module Layout

```
lifelink/
|-- config.h                  pin map + bus speeds + sample window
|-- main.c                    calls lifelink_init() then lifelink_loop()
|-- drivers/
|   |-- max30102/             FIFO drain, register I/O
|   |-- max30205/             temperature read and alert thresholds
|   `-- hm10/                 bounded UART wrapper
`-- app/
    `-- lifelink/             init + loop + HR/SpO2 math
```

`drivers/` is one static library, `app/` is another, and `main.c` links
both.
