# Board motion demo

Build a standalone Arduino sketch in this folder for the Composite Board rev C
described in `../Jul27.ino`: ESP32-S3-MINI-1, ICM-42688-P, SPI MISO 8,
MOSI 9, SCLK 10, CS 11, motor input 14 held low.

The ESP32 samples at 250 Hz on its own task and keeps a bounded history. A
self-contained web page served from flash retrieves batches over the board's
Wi-Fi access point. Browser-side quaternion fusion combines calibrated gyro
integration with accelerometer tilt correction. It removes gravity in world
coordinates and drives a damped, bounded movement effect. This is a visual
response to acceleration, not a position or distance measurement. Yaw is
relative and will drift without a magnetometer. Sensor axes are shown explicitly;
the illustrated PCB is schematic because this folder contains no board drawing.

Implementation and verification:

- [x] Write deterministic motion tests: stationary calibration, moving calibration
  rejection, 90-degree rotation, tilted gravity removal, acceleration response,
  invalid samples, clock wrap, and connection gaps.
- [x] Implement `web/motion.js` with independent fusion and packet handling.
- [x] Implement `demo.ino` with IMU identity/configuration checks, motor off,
  separate sampling task, bounded batch endpoint, and built-in Wi-Fi access point.
- [x] Implement offline canvas renderer and viewer with live/simulated modes,
  calibration, recenter, stale-data state, and an acceleration trace.
- [x] Generate `WebAssets.h` from the web source with `tools/build-assets.mjs`.
- [x] Run motion tests, build consistency and JavaScript syntax checks, try an
  ESP32 compile, and inspect the browser at desktop and mobile widths.
- [x] Document flashing, connection, axes, limitations, and hardware checks that
  require a physical board. Keep all deliverables under `demo/`.
