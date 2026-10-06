# AdvantageScope logging

`TeleopMain` records INIT, ACTIVE and STOP snapshots. Every `Robot` instance opens its own session; `Robot.write()` records one snapshot after its outputs. Other robot OpModes should set `robot.logging.mode("ACTIVE")` at START and close `Robot` at STOP. Independent tuning/example OpModes that do not instantiate `Robot` retain their existing calibration exports; they do not automatically get this full-robot log.

## Open a recorded run

1. Run `TeleopMain`. Driver Station telemetry shows logging status and the full file path.
2. Stop the OpMode. The logger attempts to drain queued samples after actuator shutdown.
3. Copy `/sdcard/FIRST/biobuzz-logs/` from the Robot Controller. For a device already connected through ADB, use `adb pull /sdcard/FIRST/biobuzz-logs ./biobuzz-logs`.
4. In AdvantageScope, open the desired `run-<time>-<unique>.csv` with **File â†’ Open Log**. Retain the matching `.csv.json` configuration and `.csv.summary.json` completion report alongside it.
5. Add Line Graph and Table tabs. Drag fields from the sidebar into the tabs. Start with the comparison groups below.

Files follow AdvantageScope's [CSV Table format](https://docs.advantagescope.org/overview/log-files/): `Timestamp` in seconds followed by named scalar columns, with JSON-encoded strings and lowercase booleans. The importer splits literal commas/newlines, so strings encode commas as `\u002c` and use JSON quote/control-character escaping; headers remain unquoted. CSV supports numeric plots and state inspection; it does not preserve binary pose structs. For the live pose view, our `Localization/Pose x`, `y`, and `heading` fields follow the [FTC Dashboard pose convention](https://docs.advantagescope.org/tab-reference/2d-field/). They use Pedro inches/radians; align the viewer's field origin and axes with `lib/field/Field.java` before comparing field graphics. A correctly aligned BIOBUZZ field asset is not bundled.

## Live AdvantageScope connection

1. Connect the computer to the Robot Controller's network and enable FTC Dashboard.
2. Set AdvantageScope's robot address to the controller address: normally `192.168.43.1` for a Control Hub or `192.168.49.1` for a phone RC. Dashboard uses port 8080; its browser page is `/dash` on that address.
3. Choose **File â†’ Connect to Robot â†’ FTC Dashboard**. INIT the OpMode to start publishing.
4. Live updates use the same field names as the file, normally at 10 Hz. `Logging/CaptureTime_s` identifies the original snapshot time; the live transport's own timeline reflects delivery. The CSV is the source for precise capture-time comparisons.

References: [AdvantageScope live sources](https://docs.advantagescope.org/overview/live-sources/) and [FTC Dashboard setup](https://acmerobotics.github.io/ftc-dashboard/gettingstarted.html). No NetworkTables server or WPILib native dependency is needed. Set `LoggingConfig.live = false` to keep file recording without publishing packets; disable Dashboard separately when you want its server disabled too.

## What to inspect

| Field group | Recorded signals and useful comparisons |
| --- | --- |
| `Session` | INIT/ACTIVE/STOP and selected alliance. |
| `Localization` | Fused and raw odometry pose, velocity, full 3Ã—3 pose covariance, position sigma, sensor health and fusion result. Compare raw/fused trajectories when vision corrects position. |
| `Vision` | Source status, frame sequence, capture time, age, and last frame's per-tag camera-space XYZ for IDs 30â€“45. XYZ is forward/left/up in inches. |
| `HIVE/Red`, `HIVE/Blue` | Independent angle, rate, age, variance, freshness; latest frame's tag count, fit RMS, proposed robot pose/HIVE angle, robot innovation score, acceptance and rejection reason. |
| `Shot` | Requested preparation/feed, cancellation, selected-cell mode, solution validity/reason, readiness blockers, target angle/rate, RPM, hood, flight time, distance/height, predicted target, muzzle position/velocity, iteration count and impact HIVE variance. |
| `Turret` | Both analog voltages, encoder disagreement, measured angle/rate, target, lag-compensated/rate-limited command, servo command, health and readiness. |
| `Flywheel`, `Battery` | Target versus single-motor RPM, commanded duty, readiness and cached voltage. |
| `Hood`, `Intake`, `Indexer` | Commanded servo position/settling intake power and indexer commanded position/calibration state. Hood position is a command, not measured feedback. |
| `Drive`, `Driver1`, `Driver2` | Manual forward/strafe/yaw commands, orientation/path flags and sticks, triggers and control buttons. Drive commands describe manual requests, not individual wheel outputs or autonomous follower commands. |
| `Loop` | Read/estimation, command and output work; work total and previous start-to-start period. |
| `Logging` | Sequence, queue occupancy, file bytes, recorded/dropped counts, sink status and previous snapshot collection duration. |

Useful graph sets:

- **Shot release:** `Shot/FeedRequested`, `Shot/Ready`, `Shot/Status`, `Shot/Blocked/*`, `Intake/CommandPower`.
- **Flywheel recovery:** `Flywheel/Target_rpm`, `Measured_rpm`, `Duty` and battery voltage.
- **Turret lag:** `Turret/Target_rad`, `Command_rad`, `Measured_rad`, `Velocity_rad_s` and feedback health.
- **Vision correction:** frame age/sequence, each HIVE's `Fit/Result`, `Fit/Accepted`, `Fit/Rms_in`, `Fit/Innovation`, fused/raw pose and covariance.
- **Prediction:** `Shot/PredictedTarget_in/*`, `LaunchVelocity_in_s/*`, `Flight_s`, HIVE rate and impact variance.
- **Performance:** work and previous period, previous snapshot duration, queue occupancy and drops. A current row's previous period covers the interval since the preceding cycle start; it includes the preceding snapshot and telemetry work.

## Interpreting data correctly

All values in a row are copied after that cycle's commands/outputs; its timestamp is the cycle start. Sensor values are cached acquisitions from the cycle, not simultaneously sampled hardware. INIT and STOP records use their snapshot invocation times, and their loop-work fields can be absent/stale relative to ACTIVE measurements.

Every potentially nonfinite numeric channel has a `/Valid` boolean. Invalid values are represented by zero **and `Valid=false`** to keep the column numeric across CSV and Dashboard. Never interpret the zero alone as a measurement. `Shot/Valid` determines whether shot fields describe a usable solution; fields can retain the last calculation while idle. HIVE freshness and `Vision/FrameSequence`/age must be checked before treating a held observation as new.

`Shot/ReadinessBlockers` uses bits 1 calibration, 2 RPM range, 4 pose uncertainty, 8 reachable angle, 16 turret, 32 flywheel, 64 hood settling, 128 hood range, 256 indexer calibration, 512 robot motion. Individual boolean channels expose the same bits. These gates are evaluated only for a valid solution during preparation with healthy localization; otherwise 1023 means not evaluated/blocked. `Shot/ReadinessEvaluated` explicitly identifies this distinction. `Shot/DwellReady` reports the additional 0.10-second all-conditions dwell. Consult `Shot/Status` and `Shot/Valid` first. A new `Vision/FrameSequence` marks when fit outcomes were evaluated; held acceptance flags do not count as additional accepted frames.

Covariance uses inchesÂ² for XY, radiansÂ² for heading and mixed units off diagonal. HIVE variance is radiansÂ². Fit RMS is camera-position residual RMS in inches. The robot innovation score is the squared Mahalanobis distance when that gate was reached; its validity flag is false when an earlier gate rejected the observation.

## Limits and failure behavior

Settings are in `config/LoggingConfig.java` and are captured at INIT:

- File/live logging enabled by default; live publication interval 100 ms.
- Queue capacity 256 snapshots. A full queue drops the newest snapshot; `Logging/Dropped` counts queue/order losses.
- File cap 64 MiB per run. Reaching the cap stops file recording and leaves live publication available. `Logging/FileDropped` counts processed snapshots not written to disk.
- Minimum free storage 128 MiB, checked at initialization and periodically by the writer. Existing logs are never automatically deleted. Copy and remove old runs yourself when needed.
- Flush approximately once per second and on close. A process crash or power loss can lose buffered data and leave a partial final CSV row; the JSON completion report may be missing.
- Stop waits up to 1.5 seconds for the worker, then requests interruption. Device I/O is not guaranteed interruptible; a stalled sink can leave an incomplete run and a timeout status. The worker never touches hardware and stops publishing further live frames after an abort.
- A file-write failure disables the file sink; a Dashboard failure disables the live sink. Neither throws into control. Snapshot construction failure disables capture and appears in telemetry. A failure opening the session prevents both sinks for that session.

The queue is bounded, not allocation-free: each snapshot copies and boxes scalar state on the OpMode thread. CSV formatting, periodic flushing and Dashboard publication happen on the worker. Measure `Logging/PreviousCapture_ms`, loop period and queue drops under real load before making overhead claims. Capture-time data review is supported; deterministic software replay, match video recording, camera images and complete hardware bus capture are not implemented.

## Validation

Desktop tests exercise typed/escaped CSV, monotonic timestamps, immutable snapshots, invalid-number flags, queue overflow under blocked storage, file/live sink isolation, size limits, live throttling and close behavior. A synthetic CSV is generated in `TeamCode/build/logging-fixtures/` by the tests. Hardware acceptance still needs a real INIT â†’ START â†’ drive/aim/feed â†’ STOP run, a live AdvantageScope connection, CSV opening in the desktop app, and an overhead comparison with logging disabled. Save the build's Git revision with trial notes; the metadata records configuration but does not claim to identify the exact source commit.

Run `python tools/verify_ascope_csv.py TeamCode/build/logging-fixtures/synthetic-ascope.csv` to check the fixture with the pinned upstream CSV decoder. This optional check needs network access and Node.js and uses a stub log receiver, not the desktop UI.

Schema 2 follows the confirmed single-motor hardware: `Flywheel/Measured_rpm` and `Flywheel/Duty` replace schema 1's left/right fields. Indexer state and overall readiness dwell are included. Older run files retain their original field names.
