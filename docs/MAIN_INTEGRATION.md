# Main integration — 2026-10-05

Merged khan-moazzin's `5c65367` and `bea78b3` into the SolversLib integration branch. The user confirmed that main's hardware definitions are correct. Both original commits remain ancestors of the merge; this is not an overwrite or squash of their history.

## Resolution decisions

| Main contribution | Integrated result |
| --- | --- |
| Single `flywheel`, forward, 28 ticks/rev, 5800 RPM, 2% tolerance | One SolversLib MotorEx and one voltage-based controller; scalar gains, 116 RPM tolerance, single-motor calibration/telemetry/log fields. SDK velocity mode is replaced by our calibrated voltage controller. |
| Axon indexer hardware and coordinated feed | Visible `subsystems/Indexer`; shared feed decision with intake, SolversLib ownership, reverse/cancel/not-ready retraction, shutdown retraction, guided endpoint tuning and logs. Main's endpoint constants are explicitly placeholders, so automatic feeding waits for calibration. |
| Hood 30:173 gearing, 500–2500 µs PWM, .110 s/60° servo travel | Mechanical slope and travel-time estimate retained; absolute launch zero still comes from calibration. Existing written-output and cumulative-small-movement readiness checks retained. |
| Turret ±170°, opposed Axons, 500–2500 µs PWM | Preserved limits in defaults, aiming and manual calibration. Actual analog feedback/lag compensation retained instead of treating commanded position as measured angle. |
| 48 in / 10.61° relative hood / 62.1% shot point | Preserved in `ShotConfig.referenceTrial`, seeded into Tune 5 with 3601.8 RPM and converted hood zero. Height and flight time still require measurement. No fabricated entries added to the predictive map. |
| Whole-shot .10 s readiness dwell | Added after mechanism/vision/pose conditions. Explicit indexer-calibration and motion gates logged. |
| 6 in/s and 15°/s stationary shooting limits | Configurable `stationaryShotsOnly`; moving-shot prediction remains the default. |
| Separate main planners and Pedro Ivy commands | Retained the requested SolversLib architecture, lowercase OpModes, existing controls and `lib` layout. Main's driver-one RB shooting would conflict with our aiming assistance; G2 triggers remain shooting controls. |
| Tag-normal HIVE tracker and fixed early-feed window | Kept independent red/blue covariance states, delayed robot fusion and flight-time prediction. Tag-normal observations are a possible future addition, not an unvalidated fallback in this merge. |
| Main calibration interface | Its useful hardware/measurement functionality is represented in the guided flywheel, hood, shot and new indexer tuners. The old Ivy OpMode is not registered alongside the new ones. |

The merge retains our logging, field geometry, calibration wizard, measured shot coverage rejection and motion compensation. It does not automatically enable calibration flags. Old generated dual-motor MechanismConfig files must be replaced by newly generated scalar-gain files before use.

## Validation

See `VALIDATION.md` for build/test results. Hardware acceptance still requires physical calibration, loaded flywheel checks, indexer travel verification and a real run. Source review and desktop tests cannot establish shooting accuracy or mechanism calibration.
