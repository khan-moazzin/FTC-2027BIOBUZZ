# Integration contract

Base: team commit d54f419. Branch: codex/solverslib-biobuzz-integration.

- SolversLib 0.3.6 owns commands and team mechanisms; Pedro 3 owns drivetrain following.
- One robot loop and one follower. Read sensors and update localization before running commands and applying outputs.
- World: Pedro field inches, +X right, +Y away from audience; heading radians CCW from +X. Robot: +X forward, +Y left, +Z up. Camera observations explicitly convert the Limelight optical frame.
- Real measured turret feedback is independent of commanded position. Both 1:1 Axon feedback channels must agree before automatic shooting.
- RED and BLUE HIVE estimates have separate angle/rate/covariance/freshness. Measurements retain acquisition time, uncertainty, and provenance.
- Configuration defaults that require hardware measurement are marked uncalibrated. Calibration exports are complete source files, with fit quality and data logs.
- Calibration is separated by system: vision, turret, drive, flywheel, hood/shooter. The vision wizard exports one VisionConfig.java.
- Default driver mapping: gamepad 1 drives/intakes, right bumper enables heading assist, manual yaw overrides; gamepad 2 prepares, selects cells and requests gated shooting.
- Changes are committed in coherent buildable steps; physical validation remains distinct from simulation and desktop tests.
