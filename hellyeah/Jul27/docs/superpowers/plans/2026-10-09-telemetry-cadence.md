# Ten-second telemetry cadence implementation plan

**Goal:** Close measurement windows every 10 seconds without pausing IMU sampling while Google Sheets uploads run.

**Architecture:** The measurement loop owns the live counters and snapshots them into a bounded FreeRTOS queue at fixed deadlines. One background worker owns HTTP requests and consumes immutable records. Network completion never sets the measurement deadline or clears live counters.

**Tech stack:** Existing ESP32 Arduino firmware, FreeRTOS task/queue, HTTPClient; no additional production libraries.

**Status:** Diagnosis and proposed plan only; firmware has not been changed or flashed.

## Evidence and limits

- `Jul27.ino:459-462` checks a 10,000 ms interval, calls synchronous `sendToGoogleSheets()`, then assigns `lastSendTime = millis()` after it returns.
- `sendToGoogleSheets()` calls `http.POST()` before clearing the counters. Request work, including connection setup and waiting for the server, runs on the measurement loop.
- The piezo revision in the 야호 project repeats the same scheduling pattern at `firmware/Jul27/Jul27.ino:560-563`.
- A deterministic timing model with request durations of 3, 5 and 4 seconds produces upload starts at 10, 23 and 38 seconds. The scheduling defect is established from source; the actual 3–5-second network latency has not been measured on the device.
- No Apps Script receiver source or device timing capture was available. Server-side append time, response time, and spreadsheet display time must be distinguished from device window boundaries.
- During a blocking request, no calls to `updateArm()` occur. On resumption, its elapsed time includes that gap, so one fresh gyro reading is integrated across an interval that was not sampled. Feedback and raw logging are also delayed.

## Constraints

- Preserve swing thresholds, radius, IMU configuration, ESP-NOW packet format, arm selection, and existing `arm`, `count`, `distance` JSON keys and units.
- Preserve the piezo and magnetometer behavior in the revised board firmware.
- Do not call the live Apps Script endpoint merely to test timing: that would create research-data rows.
- Do not promise exactly spaced spreadsheet arrivals: networking and server processing remain variable.
- Original motor and newer piezo firmware are separate deployment variants. Implement the applicable variant first; port the shared timing change to the other without replacing its hardware-specific code.
- Keep all counter mutation on the measurement loop. The HTTP worker must never access live swing counters, the web server object, SPI, or the piezo output.

## Review focus

1. Slow or stalled HTTP must not pause sensing or change measurement-window duration.
2. Wi-Fi failure or a full queue must be visible, with a bounded memory policy.
3. Clock rollover and a genuinely late measurement loop must not create bogus catch-up windows.
4. A response lost after a successful server write must not silently create duplicate research rows on retry.
5. Motor-toggle event uploads must use the same worker so their synchronous HTTP call does not retain the same sampling problem.

## Task 1: Measure and reproduce

**Files:** deployment `Jul27.ino`; proposed `tests/telemetry_timing_test.cpp` and `TelemetryTiming.h` in the same sketch tree.

- [ ] Add diagnostic output for window start/end, request start/end, HTTP result, maximum inter-sample gap, and queue depth. Log device-relative milliseconds; never print Wi-Fi credentials or the endpoint URL.
- [ ] Capture several current uploads on a physical board. Compare request duration with excess spacing. Establish whether the user's observed timestamps are serial request starts or spreadsheet timestamps.
- [ ] Add a fake-clock regression test exercising the production timing helper. With transport delays of 3000, 5000 and 4000 ms, the existing scheduling model must reproduce starts at 10000, 23000 and 38000 ms. The new schedule must retain deadlines at 10000, 20000 and 30000 ms.
- [ ] Keep backend timing diagnosis separate: only inspect the Apps Script if available; do not infer its append or timestamp behavior from the client code.

## Task 2: Separate windows from delivery

**Files:** deployment `Jul27.ino`, new `TelemetryTiming.h`, new `Telemetry.ino`.

- [ ] Define a fixed-size record with arm, count, distance in cm, boot/session ID, sequence, actual window start/end milliseconds, and nominal deadline. Capture valid UTC time separately when available.
- [ ] Initialize the first deadline after warm-up and calibration, when measurement actually starts.
- [ ] At a due boundary, copy counters and reset them immediately on the measurement loop, then enqueue the copy without waiting. Check the boundary before assigning the next acquired sample's events to a window.
- [ ] Advance from the previous deadline, not HTTP completion. Use rollover-safe 32-bit arithmetic; all scheduling intervals are well below half the clock range.
- [ ] If unrelated blocking makes a boundary late, preserve actual elapsed duration and mark it late. If multiple deadlines were missed, emit one honestly labeled extended record and advance to the next future deadline. Do not fabricate several 10-second records from one aggregate count.
- [ ] Add tests for counts arriving while an earlier record uploads, zero-swing windows, `millis()` rollover, and multi-window lateness. Verify every captured swing is assigned once.

## Task 3: Move HTTP to one worker

**Files:** new `Telemetry.ino`; deployment `Jul27.ino`; `WebUI.ino`.

- [ ] Start one lower-priority FreeRTOS uploader task and a bounded queue; check both allocations before enabling telemetry. Queue creation failure must leave sensing running and report telemetry unavailable.
- [ ] Use a queue capacity of 12 jobs as the initial limit. When full, reject the new job immediately and increment a visible dropped-job counter. Continue measuring; do not block or silently extend the current window.
- [ ] Construct JSON and create/use/destroy HTTPClient only inside the worker. Start with explicit 5-second connect and 5-second response timeouts supported by the installed core; verify their real behavior because individual phase timeouts do not constitute one total request deadline.
- [ ] Preserve the existing three measurement fields. Keep the additional timing metadata in diagnostics first; include it in persisted sheet rows only after confirming the receiver's schema and updating it if necessary.
- [ ] Replace `postToggleEvent()`'s HTTP work with an immutable queued event. Capture the toggle time when the user acts, not when the worker eventually sends it. Keep local control responsive even if the event queue is full, and report event loss.
- [ ] For the initial cadence fix, do not automatically retry ambiguous POST failures. Record failures explicitly. Reliable retries require receiver deduplication by boot/session ID plus sequence; validate the receiver before enabling that behavior.
- [ ] Test using a local mock receiver with 0, 3, 5 and 15-second delays, disconnects and response failures. Confirm no HTTP path remains on the normal measurement loop.

## Task 4: Protect and verify sample timing

**Files:** deployment `Jul27.ino`; timing tests.

- [ ] Use `uint32_t` subtraction for `micros()` elapsed time so its rollover is handled correctly. The current widening into `uint64_t` does not preserve the intended 32-bit wrap subtraction.
- [ ] Report long sampling gaps and do not integrate a single fresh sample across seconds of missing data. For an initial conservative guard, reject integration across gaps above 100 ms, cancel the incomplete swing without adding an arc/count, and log the gap. Validate this threshold on recorded movement before adopting it for trials.
- [ ] Test normal 4 ms steps, microsecond rollover, and a multi-second gap. Confirm the guard cannot manufacture a large swing from the first post-gap sample.
- [ ] Do not claim hard real-time 250 Hz sampling from this change. Raw flash writes, synchronous recalibration and other web handlers remain potential delays; measure and document them separately rather than expanding this fix into a full sampler rewrite.

## Task 5: Build and bench acceptance

- [ ] Run timing/queue regression tests and compile both left and right configurations for the installed ESP32-S3 Arduino core. Check RAM/stack usage and worker allocation results.
- [ ] On two boards, collect at least five minutes under normal network conditions and then repeat with a delayed mock server and Wi-Fi loss.
- [ ] Normal-network target: device windows close at 10-second deadlines with no accumulating drift; aim for boundary jitter below 20 ms when no manual recalibration or raw-log download is running. Report measured maximum jitter rather than assuming this target is met.
- [ ] Compare inter-sample-gap statistics with the baseline. Multi-second gaps corresponding to HTTP uploads must disappear. Check two-second ESP-NOW feedback, peer-loss handling, toggle behavior, and piezo pulse limits where applicable.
- [ ] Verify the queue-full policy and failed-upload diagnostics. Confirm the server does not reinterpret reception timestamps as acquisition-window boundaries.
- [ ] If the backend accepts timing metadata, verify persisted window sequence/start/end fields end to end. Sheet arrivals may remain irregular even when acquisition and upload scheduling are correct.

## Alternatives considered

- Moving `lastSendTime` before `sendToGoogleSheets()` removes most additive drift when each upload finishes within ten seconds, but retains multi-second sampling gaps and distorted integration. It is a temporary cadence patch, not the recommended fix for measurement data.
- Reducing the interval to 5–7 seconds merely compensates for one observed latency and fails when network conditions change.
- Moving sampling to a dedicated task like the existing demo is a larger redesign. Start by removing blocking HTTP from the measurement loop and measure remaining jitter before considering it.
