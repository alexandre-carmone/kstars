# Focus Algorithm Unit Tests

This document describes the unit tests in `Tests/ekos/focus/`.  These tests cover
the **focus algorithm implementations** in `kstars/ekos/focus/` — the
`FocusAlgorithmInterface` implementations created by `MakeLinearFocuser()`, the
focus star statistics, the Aberration Inspector analysis and the tilt plate
correction maths.  No KStars window, no INDI connection, and no camera
hardware is required.

---

## Prerequisites

- `INDI_FOUND` + `CFITSIO_FOUND` (pulled in by the parent CMakeLists guard)
- No KStars data required

---

## Running the tests

```bash
# Run all focus tests:
./build/Tests/ekos/focus/testfocus -v2
./build/Tests/ekos/focus/testfocusstars -v2
./build/Tests/ekos/focus/testaberrationinspector -v2
./build/Tests/ekos/focus/testtiltcorrection -v2

# Run a single test function:
./build/Tests/ekos/focus/testfocus L1PHyperbolaTest -v2
./build/Tests/ekos/focus/testaberrationinspector tiltPureLR -v2
```

---

## Test inventory

### `testfocus.cpp` — Linear focuser algorithm

#### `basicTest`

Runs the **Linear** focuser (`FOCUS_LINEAR`, quadratic curve fit) through a
complete simulated V-curve and exercises:

- Correct initial position calculation (respecting `maxTravel` and
  `maxPositionAllowed` limits)
- First pass: position decreases by `initialStepSize` as HFR improves
- Second pass: half step-size, terminates when HFR is within `focusTolerance`
  of the first-pass minimum
- **Retry logic:** when HFR worsens during the second pass the focuser samples
  the same position up to three times before deciding whether to continue or
  finish
- **Miss-tolerance path:** when the focuser misses the desired HFR it resets to
  a polynomial-fit position and retries; tests two and three resets are also
  verified
- **Max iterations exceeded:** after 30 steps the focuser reports failure
  (`solution() == -1`)

#### `restartTest`

Replays a real-world log sequence where the starting position was "too low" on
the V-curve.  Verifies that after several consecutive worsening HFR samples the
focuser correctly resets to a higher initial position rather than continuing
inward.

#### `L1PHyperbolaTest`

Runs the **Linear 1-Pass** focuser (`FOCUS_LINEAR1PASS`) with a hyperbola curve
fit (`FOCUS_HYPERBOLA`) through a synthetic 11-point data set.  Verifies:

- Correct initial position
- All 11 first-pass measurements are consumed without branching
- `solution()` is placed at the curve minimum
- `latestValue()`, `getMeasurements()`, `getPass1Measurements()`, and
  `getTextStatus()` return correct values
- A single additional measurement at the solution confirms `isDone()` and
  `doneReason() == "Solution found."`

#### `L1PParabolaTest`

Same structure as `L1PHyperbolaTest` but uses a parabola curve fit
(`FOCUS_PARABOLA`).  Verifies that `getTextStatus()` reports the correct curve
name.

#### `L1PQuadraticTest`

Same structure but uses a quadratic curve fit (`FOCUS_QUADRATIC`).  Verifies
that L1P with quadratic behaves identically to parabola for this symmetric
data set.

---

### `testfocusstars.cpp` — Focus star statistics

Tests the star HFR / FWHM measurement statistics used to evaluate focus quality
from a list of detected stars, including outlier rejection and weighted averages.

---

### `testaberrationinspector.cpp` — Aberration Inspector analysis

Tests `AberrationInspectorAnalysis` (`kstars/ekos/focus/aberrationinspectoranalysis.*`), the headless maths
behind the Aberration Inspector dialog and the `focus_abins_result` EkosLive message: per-tile curve fits,
results table, backfocus delta, sensor tilt and the JSON payload. The dialog delegates to this class, so these
tests also pin the numbers the dialog shows.

Fixture: a 4000 × 3000 px sensor with 4.0 µm pixels and 400 px mosaic tiles, so the tile centres are at
(±7200, ±5200) µm from the sensor centre and the tilt spans are 14400 µm (left-right) and 10400 µm (top-bottom).
Three scenarios inject known per-tile solutions (focuser steps) instead of running the solver:

| Scenario | Solutions (TL, T, TR, L, C, R, BL, B, BR) | Meaning |
|---|---|---|
| A | 1010 everywhere, centre 1000 | pure backfocus: −20 µm at 2 µm/step |
| B | left column 1005, right column 995, middle 1000 | pure left-right tilt: −20 µm over 14400 µm |
| C | top row 1012, L/R 1004, centre 1000, bottom row 996 | top-bottom tilt −32 µm plus −8 µm backfocus |

- `fitHyperbolaExact`, `fitCallbackOrderAndState` — the solver recovers exact hyperbolas per tile; the per-tile
  callback runs in tile order while the solver still holds that tile's solution (what the plot needs).
- `fitTooFewPoints`, `fitDatapointMismatch` — degenerate inputs give unfit tiles and `insufficientDatapoints` /
  `datapointMismatch` warnings instead of undefined behaviour.
- `tableRows`, `deltaMicronsNotTruncated` — table rows (deltas are tile − centre, µm not truncated), tile
  selections (all / outer corners / inner diamond) and row-to-tile mapping.
- `backfocusPure`, `adviceThresholds`, `tiltPureLR`, `tiltTBPlusBackfocus` — backfocus (distance-weighted mean
  for "all" and "inner diamond") with the ±0.9 µm advice thresholds, and tilt in µm and % slope.
- `excludeTile`, `excludeCentreFails`, `unfitTile` — excluded and unfit tiles drop out of the averages; no
  centre tile means no result.
- `optCentresWeights`, `optCentresOffsetsMissing` — optimised tile centres change the backfocus weights only,
  and are ignored (with `tileOffsetsUnavailable`) when the run provided no offsets.
- `setSolutionSeam`, `optionAndKeyStrings`, `toJsonShape` — the debug/edit seam, the mapping between
  `Options::abInsTileSelection` text, wire keys and enum values, and the camelCase JSON payload.

### `testtiltcorrection.cpp` — Tilt plate correction maths

Tests `TiltCorrectionCalc` (`kstars/ekos/focus/tiltcorrectioncalc.*`), the screw adjustment maths shared by the
Tilt Correction Advisory widget and the `tiltPlate` part of `focus_abins_result`. Inputs are the scenario-B
tilt of the fixture above (−20 µm over a 14400 µm span) on a 64 mm radius plate with 0.5 mm screw pitch.

- `threePointRelative`, `threePointPushOnly`, `threePointPlateAngle90` — 3-point plates (screws at 150°, 270°,
  30° in the front view): ±0.076980 mm on the top screws, push-only normalisation, and a 90° plate rotation.
- `fourPointRelative`, `fourPointTBOnly` — 4-point plates (45°, 135°, 315°, 225°).
- `zeroTiltAndZeroSpan` — zero tilt, unknown spans and a zero pitch never produce NaN or infinity.
- `positionIndexTable` — the 8-way rear-view screw positions and their keys.
- `etaOverride` — a connected Wanderer ETA forces the 64 mm / 0.5 mm geometry for 3-point plates only.
- `trainMapRoundTrip`, `tiltToJson` — the per-train settings map and the JSON payload.

`TiltCorrectionCalc::load()` / `save()` need the KStars user database and are covered by the manual procedure.

---

## Manual test: Aberration Inspector over EkosLive

The EkosLive commands (`focus_abins_start`, `focus_abins_get_status`, `focus_abins_get_result`,
`focus_abins_get_all_settings`, `focus_abins_set_all_settings`) and the pushed `focus_abins_status` /
`focus_abins_result` messages have no automated test: `EkosLive::Message` needs a websocket server. KStars is the
websocket **client**, so run any EkosLive-compatible local server (for example the `junos-server` relay, or a
small script answering `POST /api/authenticate` with `{"success": true, "token": "<jwt>"}` and accepting the
`/message/ekos`, `/media/ekos` and `/cloud/ekos` sockets) and point both EkosLive servers in KStars at it. The
server must send `{"type": "set_client_state", "payload": {"state": true}}` on the message socket, otherwise
KStars drops every reply.

Setup: a profile with the CCD Simulator and Focuser Simulator (absolute). Focus tab: mosaic mask, full field,
Linear 1 Pass, Hyperbola, HFR; the Aberration Inspector tool button must be enabled. Then, over the message
socket:

1. `get_states` → a `focus_abins_status` frame with `canStart: true`, `running: false`,
   `lastRunState: "none"`, `hasResult: false`. When a run cannot start, `reasons` lists codes among
   `noFocuser`, `focuserNotConnected`, `noCamera`, `cameraNotConnected`, `focuserNotAbsolute`,
   `algorithmNotLinear1Pass`, `maskNotMosaic`, `curveFitQuadratic`, `inspectorRunning`, `autofocusRunning`,
   `adjustFocusRunning`, `adaptiveFocusRunning`, `focusLoopRunning`, `buildOffsetsRunning`.
2. `focus_abins_get_result` → `focus_abins_result {"available": false}`.
3. `focus_abins_start` → `focus_abins_status` with `startAccepted: true`, `running: true`; when the run ends,
   `focus_abins_result` (nine tiles; `solution`, `deltaTicks` and `r2` match the dialog table, `backfocus` and
   `tilt` match the Analysis fields, `tiltPlate.points[].adjustmentMm` match the advisory) followed by
   `focus_abins_status` with `lastRunState: "complete"`, `hasResult: true`. The desktop dialog opens once.
4. `focus_abins_start` twice quickly → the second reply has `startAccepted: false` and `reasons` containing
   `inspectorRunning` (and `autofocusRunning` once the autofocus has begun); afterwards a plain Autofocus does
   not open the inspector.
5. Switch the algorithm to Linear → `focus_abins_get_status` reports `reasons: ["algorithmNotLinear1Pass"]`
   and `focus_abins_start` is rejected with an `error` text.
6. Stop a run → `new_focus_state {"status": "Aborted"}` then `focus_abins_status` with
   `lastRunState: "aborted"`; the previous result is still returned by `focus_abins_get_result`.
7. `focus_abins_set_all_settings {"abInsTileSelection": "innerDiamond", "abInsOptCentres": true,
   "tiltPlateType": 1, "tiltRadius": 70}` → the reply echoes the effective settings; the next run's dialog
   and result use them. Invalid values (`"tiltRadius": 0`, `"abInsTileSelection": "x"`) are ignored.
8. A build without Qt Graphs still sends `focus_abins_result`, with `dialogAvailable: false` in the status.
