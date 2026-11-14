# Execution ledger: 2026-10-09-telemetry-cadence.md

- User authorized implementation and compilation on 2026-10-09.
- Baseline: all 20 existing demo tests pass. Installed ESP32 core is 3.3.5.
- Ruling: edit the authorized working files in place; the Git root is the parent directory and the project includes untracked user files. Do not move, discard, commit or reinitialize them.
- Ruling: no device or receiver source is available. Use native C++ regression tests and four ESP32 builds; leave physical timing and receiver behavior explicitly unverified.
- Ruling: preserve the payload schema; record window timestamps locally and through status diagnostics. No automatic retries without server deduplication.
- Ruling: port only changed gait firmware files into the piezo project after verification; preserve its sensor/piezo logic and historical validation records.
- Pre-flight: window snapshots feed the queue by value; worker has no access to live counters. Toggle events share the worker, retaining action-time timestamps.
- Implemented: fixed deadlines and immutable snapshots; a bounded FreeRTOS HTTP worker; nonblocking toggle logging; status/error diagnostics; rollover-safe elapsed time and the 100 ms gap guard.
- Regression evidence: timing test first failed for the missing production header, then passed. Production worker passes 10 host scenarios (allocation failures, overflow, slow upload, toggle timestamp, offline, redirect, HTTP error, timeout, begin failure). All 20 existing demo tests still pass.
- Independent read-only code review found no actionable correctness issues in task/queue ownership, status locking, rollover, deadline scheduling or variant integration.
- First ESP32 build: original motor/right configuration passed on core 3.3.5, 1,140,771 bytes flash and 48,320 bytes static RAM. Remaining three configurations are being compiled.
- Ruling: worker priority is 1 on the core opposite the Arduino loop on dual-core ESP32-S3, rather than priority 0; blocking network APIs yield to the scheduler, while priority 0 could starve the uploader. On a single-core configuration it shares core 0.
- Motor/left compile and link passed, but the first command's `--output-dir` triggered a broken ESP32 Windows export hook (missing default `build/esp32.esp32.esp32s3` output directory). Removed that CLI option and copied artifacts after compilation. Rerun exited 0: 1,151,819 bytes flash, 48,320 bytes static RAM. No source fix was needed.
- Final build verification: all four CLI builds exited 0 for ESP32-S3/core 3.3.5. Motor R/L: 1,140,771 / 1,151,819 bytes flash, 48,320 bytes static RAM. Piezo R/L: 1,167,171 / 1,178,379 bytes flash, 49,792 bytes static RAM.
- Final test verification: native timing suite and all 10 production-worker scenarios pass; motor demo 20/20 and piezo demo 21/21 pass. `git diff --check` passes.
- `TELEMETRY_BUILD_RESULTS.json` records exact source/binary hashes and build logs. Binaries are retained under `.build/artifacts/` and no firmware was flashed.
- Applied the verified right-default piezo sketch, shared telemetry sources, documentation and tests to the 야호 project. Checked unchanged baseline hashes before overwrite and matched every copied production file against the successful build manifest afterward. Hardware-specific helpers and historical validation records are preserved.
- Implementation and compile work complete. Device timing capture, queue saturation under real Wi-Fi/TLS load, sound/radio bench tests, and receiver-side row verification remain unperformed because no board session or Apps Script source was available. They are not claimed by the automated checks.
