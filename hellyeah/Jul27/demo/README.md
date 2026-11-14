# IMU board demo — wired USB

The default firmware streams motion over USB serial. **Wi-Fi stays off.**
The local webpage can show real readings through Web Serial, or simulated motion.

## Run the wired demo

1. Open `demo/demo.ino` as its own sketch in Arduino IDE.
2. Select **ESP32S3 Dev Module** and the board's port. Keep the flash settings
   that work with your original sketch. For the board's native USB connector:
   - **USB CDC On Boot: Enabled**
   - **USB Mode: Hardware CDC and JTAG**
   - **Upload Mode: UART0 / Hardware CDC**
   If using an external USB-to-UART adapter instead, leave CDC disabled and use
   the original UART configuration. The demo uses **460800 baud**.
3. Upload, then press Reset if the board does not start. Uploading replaces the
   current gait firmware; upload the parent `Jul27.ino` to restore it later.
4. **Close Arduino Serial Monitor and Serial Plotter.** Only one application can
   hold the serial port. Keep the board attached with a USB data cable.
5. In a terminal in this `demo` folder, run:

   ```powershell
   node tools/serve.mjs
   ```

6. Open **http://127.0.0.1:8127/** in desktop **Chrome or Edge**.
   Use a normal browser window if the Codex in-app browser does not offer a
   working serial-device picker. No Wi-Fi network change or internet is needed.
7. Click **Connect USB**, select the ESP32's serial/COM port, and connect.
8. Keep the board still for two seconds, then rotate or move it.

The updated demo firmware must be uploaded: earlier demo and gait firmware do
not provide this USB packet stream. The page does not flash the board for you.
The browser remembers permission, but this demo asks you to select the port on
connection. Disconnect USB in the page before uploading another sketch.

## If no live data appears

- **No device picker / unsupported browser:** use desktop Chrome or Edge at the
  localhost address above. Do not open `index.html` directly from disk.
- **Port busy:** close Serial Monitor, Serial Plotter, other tabs using the board,
  and any serial terminal. Click Connect USB again.
- **Connected but waiting:** confirm the new `demo.ino` is uploaded and native
  USB CDC is enabled. Check that the cable carries data. If necessary, disconnect
  in the page, open Serial Monitor at 460800 baud, reset, and look for the IMU
  identity message and JSON packets. Close the monitor before reconnecting.
- **IMU error:** `WHO_AM_I` must be `0x47`. Check the existing SPI wiring below.
- **Board reset / unplugged:** reconnect using Connect USB. Calibration starts
  again on a new boot or long sample gap.

## Controls

- **Connect USB / Disconnect USB:** open or release the wired connection.
- **Simulated motion:** test the viewer without hardware; clearly labeled.
- **Wi-Fi:** optional legacy connection, described below.
- **Recenter view:** set the current orientation as the displayed zero pose.
- **Calibrate gyro:** hold still for two seconds to measure three-axis gyro bias.
- **Show movement / Movement gain:** toggle and scale the translation effect.
- **Drag / scroll / Reset camera:** orbit, zoom, and reset the viewing angle.

## Hardware

### Board axis mapping

The webpage converts raw chip axes to board axes before calibration and fusion:
`board = [sensor Y, -sensor X, sensor Z]` for both acceleration and gyro.
With the component side facing you and USB at the bottom, board +X is right,
+Y is up (away from USB), and +Z points toward you. This follows the supplied
PCB image (IMU pin-1 dot at upper right) and TDK DS-000347 section 10.1,
Figure 16. The simulation already uses board coordinates.
This browser-side correction requires refreshing the local page and reconnecting
USB, with no firmware upload. Raw serial packets keep the original chip axes.

Reuses the pin map and SPI transport from the parent `Jul27.ino`:

| Connection | GPIO |
| --- | ---: |
| ICM-42688-P MISO | 8 |
| MOSI | 9 |
| SCLK | 10 |
| CS | 11 |
| Motor input, held LOW | 14 |

ESP32-S3-MINI-1 + ICM-42688-P. SPI mode 3, 8 MHz. The sensor uses 1 kHz ODR,
±16 g (2048 LSB/g) and ±2000 degrees/s (16.4 LSB/dps); software samples at 250 Hz.
No additional Arduino libraries are needed. The original sketch files are unchanged.

## What the model shows

The model approximates the supplied board images: square PCB, ESP32 shield,
USB-C, two switches, IMU and battery connector. Antenna and tiny parts are omitted.
The renderer uses per-pixel depth checking so the board cannot incorrectly paint
across raised components. Dimensions and the IMU-to-PCB mounting transform have
not been measured. Arrows represent IMU axes; world Z is upward.

Rotation uses calibrated gyro integration plus accelerometer tilt correction.
Yaw is relative and may drift. Recalibrate after warm-up if necessary.
**Movement is a damped response to gravity-corrected acceleration, not absolute
position tracking.** It returns to center and does not measure distance traveled.
Slow constant-speed motion cannot be reliably recovered from this IMU alone.

## Transport and development

The sampler runs independently of serial writes. Every approximately 20 ms,
firmware writes one newline-terminated JSON packet at 460800 baud:

```json
{"boot":123,"latest":5,"dropped":false,"samples":[[5,20000,0,0,1,0,0,0,25]]}
```

Each sample is `[sequence, micros, ax, ay, az, gx, gy, gz, temperature]`, with
acceleration in g, gyro in degrees/s, and temperature in Celsius. A 128-sample
ring buffer covers brief stalls. Boot IDs, sequence numbers, dropped-history
flags, and sensor timestamps handle resets and missing data. Browser reads are
chunked; the parser buffers partial lines and ignores startup log text.

```powershell
node --test tests/*.test.mjs
node tools/build-assets.mjs
node tools/build-assets.mjs --check
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc .
```

Node.js 18+ is sufficient, with no `npm install`. Edit `web/` sources; the local
server serves them directly. Rebuild `WebAssets.h` for optional Wi-Fi uploads.
Tests cover motion math, per-pixel depth, split serial packets, malformed lines,
port release, and canceling a pending device selection. Actual cable/IMU behavior
still needs checking on your physical board.

## Optional Wi-Fi mode

Set `DEMO_WIFI` to `1` near the top of `demo.ino` and upload to restore the AP.
Join `BoardMotion-XXXX`, password `motiondemo`, and open http://192.168.4.1/.
The page automatically selects Wi-Fi when served by the board. With the default
`DEMO_WIFI=0`, no AP is started and the embedded web server is excluded.

References: [Web Serial API](https://developer.chrome.com/docs/capabilities/serial)
and [Espressif USB CDC setup](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html).
