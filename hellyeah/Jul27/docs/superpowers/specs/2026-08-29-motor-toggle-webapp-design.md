# Mobile Web Control for Motor Toggle, Raw IMU Logging, and Drift Mitigation

## Context

Two ESP32-S3 boards (`Jul27.ino`, one per arm) track arm swings via an ICM-42688-P
IMU, drive a per-arm vibration motor when that arm lags its peer, and post
windowed swing stats to Google Sheets. For controlled research trials, the
motor's haptic feedback needs to be toggled on/off from a phone without
touching a physical button, and marked in the data so on/off periods can be
compared after the fact.

Two follow-on needs surfaced during design: (1) a way to capture raw IMU
samples for short tuning sessions to dial in the swing-detection constants,
and (2) mitigating gyro drift, which the boards are already somewhat prone to.

## Goals

- Toggle the vibration motor on both boards together, from one phone page, on
  the local WiFi network only.
- Log every toggle event with a timestamp to the existing Google Sheet, so
  on/off periods can be lined up with swing data during analysis.
- Show live status for both boards on the same page (not just the toggle
  state) — connectivity, motor state, temperature, raw-log state.
- Support short (~1-2 min) raw IMU capture sessions, downloadable from the
  phone, for offline threshold tuning. Not intended to run during full
  research trials.
- Reduce gyro drift risk without requiring the study participant to stop and
  hold still mid-trial.

## Non-goals

- Remote control from outside the local WiFi network (cloud relay).
- Continuous raw-sample logging for the full duration of a research trial.
- Automated temperature-vs-bias characterization/compensation (ruled out as
  disproportionate effort for uncertain benefit — see Drift Mitigation).

## Architecture

Both boards run a lightweight `WebServer` (ESP32 Arduino core, synchronous —
no new library dependency) and advertise a unique mDNS hostname:
`bb-left.local` / `bb-right.local`, derived from the existing `ARM_CHAR`. The
LEFT board is the fixed "master" and additionally serves the combined control
page. Serial-printed IP addresses remain as a fallback in case mDNS
resolution is flaky on a given phone/network.

Motor mode and raw-log status are exchanged the same way peer swing data
already is: as extra fields piggybacked onto the existing `PeerMsg` struct
broadcast over ESP-NOW every `FB_WINDOW_MS` (2s). No new radio protocol is
introduced. The LEFT board is the sole authority for motor mode — the RIGHT
board always adopts whatever value it last received from a peer packet
tagged `arm == 'L'`; it never originates its own value.

### Per-board HTTP endpoints (both boards)

| Endpoint | Method | Purpose |
|---|---|---|
| `/state` | GET | JSON status: `{arm, motorEnabled, peerFresh, rawLogging, rawLogBytes, tempC, uptimeMs}`. CORS-open (`Access-Control-Allow-Origin: *`) so the master page can fetch it cross-origin from the other board's hostname. |
| `/recalibrate` | POST | Soft re-zero of gyro bias in the background. Does **not** reset swing counts, `g_angle`, or any in-flight windows — see Drift Mitigation. |
| `/rawlog/start` | POST | Begin writing raw IMU samples to a LittleFS file. |
| `/rawlog/stop` | POST | Stop writing, finalize the file. |
| `/rawlog/download` | GET | Stream the file for browser download. |
| `/rawlog/clear` | POST | Delete the file to free flash space. |

### LEFT-board-only endpoints

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Control page: HTML + inline JS. Big motor ON/OFF toggle; status tiles for both arms (client-side JS polls this board's own `/state` plus `http://bb-right.local/state`); raw-log start/stop/download/clear controls for each board (JS talks directly to each board's own hostname — downloads go phone-to-board directly, not proxied through the master). |
| `/toggle` | POST | Flips `g_motorEnabled` locally, includes it in the next `PeerMsg` broadcast, and fires one synchronous HTTP POST to the existing Google Apps Script (`scriptURL`) logging `{"event":"motor_toggle","state":"on"/"off","ts":<millis or ISO time>}` as a marker row. |

## Motor Toggle Behavior

- `g_motorEnabled` defaults to `true` at boot on both boards (normal feedback
  operation unless a researcher explicitly disables it).
- Swing detection, counting, and the existing 10s Sheets telemetry
  (`sendToGoogleSheets`) are **untouched** — they keep running identically
  regardless of motor mode. Only the `analogWrite(PIN_VIBE, ...)` call inside
  `feedbackTick()` is gated behind `g_motorEnabled`. This preserves identical
  measurement fidelity in both the motor-on and motor-off conditions, since
  the only thing that differs is the haptic cue itself.
- If the peer goes stale (`FB_PEER_TIMEOUT` exceeded), the motor already
  forces off via existing logic — this is unchanged and composes fine with
  the new gate.
- The `/toggle` handler's Sheets POST is synchronous and may block the
  board's loop for under ~1s. This only happens at the moment of a button
  press, not during a trial, so a brief IMU-sampling gap right then is an
  acceptable trade-off.

## Status Panel

The control page polls both boards' `/state` every ~1-2s and renders per-arm
tiles showing: motor state (on/off, and whether it's stale/forced-off due to
a dead peer), last-seen/connectivity freshness, current temperature, and
raw-log state (idle / recording / bytes captured). This gives the researcher
a way to visually confirm both boards actually converged on the same motor
state before starting a trial, rather than trusting the toggle blindly.

## Raw IMU Logging

Intended for short, dedicated tuning sessions (~1-2 min), run separately from
real research trials — not continuously.

- Each board writes raw samples to its own LittleFS file
  (`/rawlog.csv`) as: `ts_ms,ax,ay,az,gx,gy,gz,tempC` per row, at the existing
  `SAMPLE_RATE_HZ` (250Hz).
- Started/stopped independently per board via `/rawlog/start` and
  `/rawlog/stop`, triggered from the buttons on the master page (which call
  each board's own hostname directly).
- A safety cap (time-based, e.g. auto-stop after 3 minutes, and/or a byte-size
  ceiling) prevents an unattended session from filling flash. Exceeding the
  cap stops logging and the state reflects it, but never crashes the sketch.
- `/rawlog/download` streams the CSV directly to the phone's browser for
  offline analysis (e.g. in Python/Excel) to tune `GYRO_THRESH_FWD`,
  `ACCEL_MAG_THRESH_DELTA`, `RADIUS_DEFAULT`, etc.
- `/rawlog/clear` deletes the file once downloaded, freeing space for the
  next session.
- Raw logging and the normal swing-detection/telemetry pipeline run
  concurrently without conflict — logging just taps the same `readSample()`
  call already happening every loop iteration.

## Drift Mitigation

Three complementary, low-effort layers — chosen over a full
temperature-characterization curve, which would require a per-unit
calibration campaign (doesn't transfer across boards, invalidated by any
hardware swap) for benefit that mostly matters during continuous non-stop
motion with zero idle windows, a case the runtime tracker below already
handles reactively.

1. **Tightened runtime bias tracker.** The existing "idle" gate
   (`fabsf(wy) < WY_IDLE`) that nudges `g_bias` only checks the swing-axis
   gyro rate. Add an accel-magnitude gate (`aMag` within a tolerance of 9.8
   m/s²) so bias only adapts when the arm is actually near-stationary, not
   just coincidentally near a zero-crossing mid-swing. Reduces false
   adaptation during motion.
2. **Warm-up-aware initial calibration.** Self-heating after power-on can
   shift gyro bias for the first tens of seconds. Rather than locking in the
   2s boot-time calibration immediately, use the temperature reads already
   available (`readSample().tempC`) to wait until temp readings stop
   climbing (e.g. change below a small threshold across a few consecutive
   1s checks) before running `calibrateGyro()`, capped at a timeout (e.g.
   30-45s) so boot never hangs if temperature doesn't fully plateau. This
   uses time already being spent during the WiFi/ESP-NOW connect sequence —
   no added wait for the person wearing the board.
3. **Remote soft-recalibrate.** The `/recalibrate` endpoint re-zeros
   `g_bias` the same way `calibrateGyro()` does, but explicitly does **not**
   reset `g_swingCount`, `g_swingDistanceSum`, `g_fbCount`, `g_fbDist`, or
   `g_angle`'s in-progress swing state. Gives the researcher an escape hatch
   if drift is visually apparent (e.g. via the status panel or Serial log)
   without discarding trial progress — used only if needed, not routinely.

## Data Flow Summary

```
IMU sample (250Hz) ──┬─→ swing detection (unchanged) ─→ 10s Sheets telemetry (unchanged)
                      └─→ raw-log file (only while /rawlog/start is active)

Motor toggle (phone) ─→ LEFT board /toggle ─┬─→ g_motorEnabled (local, immediate)
                                             ├─→ PeerMsg broadcast (ESP-NOW, ~2s cadence) ─→ RIGHT board adopts
                                             └─→ Sheets POST: {event, state, ts} (one-off marker)

Status panel (phone) ─→ fetch bb-left.local/state + bb-right.local/state ─→ render tiles (poll ~1-2s)
```

## Error Handling / Edge Cases

- **mDNS fails to resolve on a given phone/network:** Serial log still prints
  each board's DHCP IP at boot as a fallback the researcher can type in
  directly.
- **Peer link drops mid-trial:** existing `FB_PEER_TIMEOUT` behavior
  (force motor off) is unchanged; the status panel will show the RIGHT
  tile going stale so this is visible rather than silent.
- **Raw log fills flash or hits the time cap:** logging auto-stops; state
  reflects "stopped (cap reached)" rather than silently truncating without
  indication.
- **Toggle's Sheets POST fails (no internet at that instant):** the local
  motor-mode change and ESP-NOW sync still take effect immediately;
  only the analysis marker row is lost for that event. Not retried, since a
  missed marker doesn't corrupt other data.

## Testing / Validation Plan

- Bench test: verify `/toggle` on the left board flips its own motor within
  one press, and the right board's motor follows within ~2s (one
  `PeerMsg` cycle).
- Verify the status panel correctly shows a RIGHT-board disconnect (e.g.
  power off the right board mid-session) as stale rather than stuck showing
  the last good state indefinitely.
- Verify a Sheets marker row appears for each toggle press with a sane
  timestamp.
- Run a raw-log capture session end-to-end: start, generate motion, stop,
  download the CSV, confirm sample count roughly matches
  `SAMPLE_RATE_HZ * duration`, and that a triggered cap correctly halts
  logging.
- Compare bias stability with and without the accel-gated idle check and the
  warm-up-aware calibration, using a bench IMU (stationary) sitting through a
  fresh power cycle, to confirm the warm-up delay actually reduces initial
  bias drift and doesn't introduce a regression (e.g. never calibrating due
  to a timeout edge case).
