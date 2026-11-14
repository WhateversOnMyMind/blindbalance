# BlindBalance workspace guide

This folder is a local Arduino and project workspace. It contains several generations of firmware, experiments, and local checkouts of the B.R.P website/backend. The similarly named sketches are not interchangeable; check the section below before opening or flashing one.

## At a glance

| Path | Purpose |
| --- | --- |
| `Jul27/` | Original motor firmware, its telemetry and tests, and a separate browser demo. |
| `.brp-hosted-control/` | Local B.R.P site, Cloudflare Worker, and hosted piezo/motor firmware checkout. Hidden and ignored by the workspace `.gitignore`. |
| `.brp-cloud-history/` | Another local B.R.P checkout/worktree used for cloud-history work. It contains overlapping site, Worker, and firmware files, plus the `0002_measurements.sql` history migration. |
| `BoardDiagnostics/` | Small board-revision-C hardware check: bus scan, IMU identity check, and piezo pitch test. |
| `UpdatedHardware/` | Earlier single-arm swing tracker for a different XIAO ESP32S3 + ICM-20948 setup. |
| `sketch_jun12a/` | Early ICM-20948 I2C identity/register diagnostic. |
| `sketch_oct02a/` | Older two-output motor/pin test sketch. |
| `Test/Test/` | Minimal motor-output timing test. |
| `hellyeah.ino` | Legacy single-file tracker prototype. |
| `Jul27 (2).zip` | Archived copy of an earlier Jul27 project; inspect/extract a copy rather than editing the archive. |

The root `.gitignore` excludes `.brp-hosted-control/`. The two hidden B.R.P folders are separate checkouts and can contain duplicate or different revisions of the same files. Before changing or publishing hosted code, check which checkout and branch is current; do not assume edits in one are present in the other.

## Original firmware: `Jul27/`

This is the original motor project for the Composite Board rev C. Its `Jul27.ino` uses GPIO 14 for motor PWM, sends per-board ten-second summaries to the Apps Script/Google Sheets endpoint, and exchanges arm data over ESP-NOW. It is separate from the hosted Cloudflare firmware under `.brp-hosted-control/firmware/`.

| File or folder | What it does |
| --- | --- |
| `Jul27.ino` | Main ESP32-S3 sketch: IMU sampling, swing detection, pairing, balance feedback, Wi-Fi, and the main loop. Select the board's arm with `ARM_IS_LEFT`. |
| `WebUI.ino` | Local ESP32-hosted configuration/status endpoints and web controls for this original firmware. |
| `RawLog.ino` | Raw sensor-capture controls and logging endpoints. |
| `Telemetry.h`, `Telemetry.cpp` | Queued background delivery of measurement and motor-toggle records. |
| `TelemetryTiming.h` | Measurement-window and timing helpers used by telemetry. |
| `AlertWindow.h` | Tracks whether feedback output was commanded during a measurement window. |
| `TELEMETRY.md` | Explains ten-second windows, uploads, diagnostics, failure behavior, and verification. |
| `FIRMWARE_SOFTWARE_GUIDE_KO.md` | Korean firmware/software operating and setup guide. |
| `ALERT_BUILD_RESULTS.json`, `TELEMETRY_BUILD_RESULTS.json` | Saved build/verification reports for earlier revisions; not live device state. |
| `apps-script/Code.gs` | Google Apps Script receiver for the original firmware's measurements and toggle events. |
| `apps-script/README.md` | Instructions and data meanings for updating that Apps Script deployment. |
| `tests/run-tests.ps1` | Builds and runs the native C++ telemetry and alert-window tests. |
| `tests/telemetry_timing_test.cpp` | Tests measurement-window timing behavior. |
| `tests/telemetry_worker_test.cpp` | Tests the upload queue/worker against local fake Wi-Fi and HTTP interfaces. |
| `tests/alert_window_test.cpp` | Tests feedback-command tracking per measurement window. |
| `tests/fakes/` | Minimal Arduino, FreeRTOS, Wi-Fi, and HTTP stand-ins for native tests; not ESP32 firmware libraries. |
| `demo/` | Independent motion visualization demo; see below. |
| `docs/superpowers/specs/`, `docs/superpowers/plans/` | Design notes and implementation plans for local controls and telemetry work. |
| `.build/` | Generated and cached build files, including compiled copies. Treat as disposable build output, not source of truth. |

### `Jul27/demo/`

This is a standalone motion-visualization demo, not the hosted BlindBalance control website and not the original production firmware.

| File or folder | What it does |
| --- | --- |
| `demo.ino` | Demo ESP32 sketch; samples the IMU and serves data from the board. |
| `WebAssets.h` | Generated C++ header containing the demo web files for serving from flash. |
| `web/index.html`, `web/style.css` | Demo page structure and styling. |
| `web/app.js` | Browser-side app wiring, modes, and controls. |
| `web/motion.js` | Browser-side motion processing and sensor-axis logic. |
| `web/renderer.js` | Canvas visualization. |
| `web/serial.js` | Serial input/packet handling for the demo. |
| `tools/build-assets.mjs` | Rebuilds `WebAssets.h` from the web sources. |
| `tools/serve.mjs` | Serves the demo locally during browser development. |
| `package.json` | Demo scripts and Node test configuration. |
| `PLAN.md`, `README.md` | Demo design, setup, operation, and limitations. |
| `tests/` | Tests for motion math, board axes, rendering, and serial parsing. |

## Hosted B.R.P checkout: `.brp-hosted-control/`

This hidden checkout holds the website and Cloudflare backend that serve the hosted control page, plus source for two board variants. It is ignored by the parent workspace's `.gitignore`; changes here are not automatically changes to a parent Git repository.

| File or folder | What it does |
| --- | --- |
| `README.md` | B.R.P site overview and links to site/firmware setup information. |
| `package.json` | Node scripts for building the static site and running its tests. |
| `netlify.toml` | Netlify production build settings for generating and publishing `dist/`. |
| `tools/build.mjs` | Copies the explicitly approved site files into the generated `dist/` folder. |
| `tools/README.md` | Site build and deployment notes. |
| `tests/` | Website, control API/state, and end-to-end integration tests. |
| `LICENSE` | Repository license. |

### Website source: `B.R.P Website/`

| File or folder | What it does |
| --- | --- |
| `home.html`, `homestyle.css` | Main B.R.P landing page and its styles. |
| `signup/` | Signup page (`index.html`), behavior (`main.js`), and styles (`style.css`). |
| `checkin/` | Event check-in page (`index.html`), behavior (`checkin.js`), and styles (`style.css`). |
| `control/` | Hosted BlindBalance controls: page (`index.html`), client behavior (`control.js`), API requests (`api.mjs`), state model (`state.mjs`), runtime API URL configuration (`config.mjs`), and styles (`style.css`). |
| `data/index.html` | Data page. |
| `assets/` | Images used by the site. |
| `netlify.toml` | Netlify settings when building from this site directory. |
| `meta.env` | Local site metadata/configuration file; check before sharing because environment files can contain private values. |
| `.idea/` | IDE project settings; not website runtime code. |

### Cloudflare Worker: `cloudflare/`

| File or folder | What it does |
| --- | --- |
| `src/worker.mjs` | Cloudflare API implementation for device pairing, settings/commands, status, and measurement history. |
| `migrations/0001_control.sql` | Initial D1 database schema for control and device data. |
| `wrangler.jsonc` | Local/template Wrangler configuration. |
| `wrangler.production.jsonc` | Production-specific Wrangler configuration if present locally; check its ignore status and keep account-specific values private. |
| `package.json`, `package-lock.json` | Worker scripts and pinned tool dependencies. |
| `test/` | Worker tests and local D1/R2 test harness. |
| `tools/provision.mjs` | Generates local pair/device/operator provisioning artifacts. |
| `tools/smoke.mjs` | Runs an authenticated smoke check against a dedicated test pair. |
| `README.md` | Backend setup, API contract, deployment, security, and test notes. |
| `build/`, `.wrangler/`, `provisioning/` | Generated build/cache or private provisioning output when present; do not publish or commit secrets. |

### Hosted firmware: `firmware/`

| Path | What it does |
| --- | --- |
| `Jul27/` | Hosted **piezo** variant. Main sketch is `Jul27.ino`; it uses GPIO 14 for piezo output and Cloudflare control/telemetry. |
| `Jul27Motor/` | Hosted **motor** variant. Main sketch is `Jul27Motor.ino`; motor feedback logic is in `MotorFeedback.h`. |
| `README.md` | Firmware variants and setup overview. |
| `relocate-2026-10-24.ps1`, `restoration-manifest.json` | One-time firmware relocation/restoration helper and its file manifest; not part of normal flashing. Review carefully before running. |

Each hosted firmware directory contains its own Cloudflare control files, settings/command logic, peer protocol/authentication, capture/logging code, example Wi-Fi/cloud headers, and tests. The real `WifiConfig.h` and `CloudConfig.h` files are local/private configuration when present; `*.example.h` files are safe templates. Keep device/operator credentials and Wi-Fi passwords out of Git, screenshots, and public site builds.

The separate `.brp-cloud-history/` checkout has an additional `cloudflare/migrations/0002_measurements.sql` migration for ten-second measurement history. Confirm which checkout matches the deployed website/API before making a change that must be published.

## Other local sketches

| Path | What it does |
| --- | --- |
| `BoardDiagnostics/BoardDiagnostics.ino` | Board rev C check: scans I2C, checks the SPI IMU identity, and plays piezo test tones on GPIO 14. |
| `UpdatedHardware/UpdatedHardware.ino` | Separate one-device swing tracker for Seeed XIAO ESP32S3 + ICM-20948. It is not the rev C hosted piezo/motor firmware. |
| `sketch_jun12a/sketch_jun12a.ino` | Small I2C register/WHO_AM_I diagnostic for an ICM-20948. |
| `sketch_oct02a/sketch_oct02a.ino` | Early MPU6050/motor prototype using two output pins and Google Apps Script. |
| `Test/Test/Test.ino` | Minimal alternating output-pin test sketch. |
| `hellyeah.ino` | Earlier single-file MPU6050 tracker prototype with legacy web/upload logic. |

Some old prototype sketches contain credentials directly in source. Treat them as private; before sharing or publishing them, remove those values and rotate any credentials that have been exposed.

## Which files should I use?

- For the original motor device and its local controls, start with `Jul27/`.
- For hosted piezo or motor device firmware, start with `.brp-hosted-control/firmware/Jul27/` or `.brp-hosted-control/firmware/Jul27Motor/`, after confirming that this checkout is the one you intend to update.
- For the website and API, inspect `.brp-hosted-control/B.R.P Website/` and `.brp-hosted-control/cloudflare/`.
- For board electrical checks, use `BoardDiagnostics/`.
- Do not flash the historical prototype sketches just because their names or dates look similar.

