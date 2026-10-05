# Code structure

The editable robot surface is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`.

| Location | What belongs here |
| --- | --- |
| `Robot.java` | Construct hardware once; acquire/fuse, run commands, then apply outputs; clean up partial initialization and shutdown. |
| `Constants.java` | Alliance and Pedro/Pinpoint configuration produced by tuners. |
| `config/` | Copy generated `VisionConfig`, `MechanismConfig`, and `ShotConfig` here. Their package names and export destinations are unchanged. `LoggingConfig` sets file/live recording limits. |
| `subsystems/` | Robot-specific behavior, hardware names, control policy and SolversLib requirements. |
| `opmodes/TeleOp.java` | Driver controls and the competition loop. Driver Station name remains `TeleopMain`. |
| `opmodes/tuning/` | All six guided calibration/verification OpModes. Driver Station names remain unchanged. |
| `lib/control/` | Tested angle/target math, shot solver/map, readiness, voltage cache and profiling. |
| `lib/vision/` | Limelight observations, transforms, per-HIVE state and pose fitting. |
| `lib/logging/` | Immutable snapshots, bounded asynchronous CSV recording and live AdvantageScope via FTC Dashboard. |
| `lib/localization/` | Buffered delayed fusion and sensor health gates. |
| `lib/math/`, `lib/field/` | Numerical helpers and sourced field geometry. |
| `lib/calibration/` | Shared calibration lifecycle, fit and configuration export support. |
| `lib/pedro/` | Pedro tuner registration, procedures and disabled reference examples. |

`lib` means reusable support that normally does not need editing when tuning or changing driver behavior. It does **not** mean hardware-verified or immutable. Changes to these algorithms should include regression checks. Subsystems stay visible because they describe this robot and are likely to change as the mechanism design changes.

Tests remain in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode`. SDK scaffolding in `FtcRobotController`, Gradle build files, and binary dependencies keep their required locations. No duplicate copy of the old support packages is kept. External SolversLib/Pedro dependencies remain pinned rather than copied into `lib`.

The old `control`, `vision`, `localization`, `math`, `field` and `pedro` packages now have a `lib` prefix. Old `calibration` OpModes moved to `opmodes.tuning`; their shared helpers moved to `lib.calibration`. The `OpModes` package is now lowercase `opmodes`. Update imports in any private code outside this checkout accordingly.
