# LifeLink

Wireless vitals monitor on a Raspberry Pi Pico (RP2040). Reads heart rate and
SpO2 from a MAX30102 and body temperature from a MAX30205, then streams the
readings over USB serial and over an HM-10 Bluetooth LE module at the same time.

```
HR=72 SpO2=98 BodyT=36.50C
```

All three drivers are written from scratch on the Pico SDK. Each sensor has its
own I2C controller. Wiring is in `connections.txt`, and the full behaviour is in
`FUNCTIONALITY.md`.

## Build

Needs the Pico SDK (`PICO_SDK_PATH` set).

```sh
mkdir -p build && cd build
cmake ..            # or: cmake -DHM10_PAIR_PIN=123456 ..  to set the BLE PIN
make
```

Hold BOOTSEL, plug the Pico in, and copy `build/lifelink.uf2` onto the RPI-RP2
drive. A prebuilt `build/lifelink.uf2` is in the repo.

## Tests

The heart rate and SpO2 math runs on a PC, no Pico needed:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```
