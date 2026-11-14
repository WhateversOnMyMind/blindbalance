# Hosted piezo firmware

This is a source copy of the hosted piezo firmware from the B.R.P repository's
`firmware/Jul27/` directory. Open `Jul27/Jul27.ino` as the Arduino sketch; keep
the adjacent headers, `.cpp` files, and `.ino` tabs in the same sketch folder.

This is the piezo version for the hosted BlindBalance control page. It uses
Cloudflare for device control and capture uploads. It does not use Google Sheets.
The original motor firmware remains in `../Jul27/`.

## Before building

1. Copy `Jul27/WifiConfig.example.h` to `Jul27/WifiConfig.h` and enter the local
   Wi-Fi or hotspot settings.
2. Create `Jul27/CloudConfig.h` from `Jul27/CloudConfig.example.h` and enter the
   correct pair ID, device key, and peer settings for this board. Keep both
   local config files private; they are intentionally not included here.
3. Set `ARM_IS_LEFT` in `Jul27/Jul27.ino` for the board being built.
4. Follow `Jul27/CLOUD_CONTROL.md` for Cloudflare setup and `Jul27/TELEMETRY.md`
   for measurement and capture behavior.

The source tests are in `Jul27/tests/`; `run-tests.ps1` runs native logic tests.
`build-cloud.ps1` checks that the full sketch links for ESP32-S3. These scripts
do not flash a board.

The B.R.P repository remains the website/backend source of truth. When updating
the piezo firmware here, compare it with `B.R.P/firmware/Jul27/` so the two
copies do not drift.
