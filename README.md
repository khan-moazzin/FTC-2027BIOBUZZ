# BioBuzz — FTC Team 26282

Robot code for khan-moazzin/FTC-2027BIOBUZZ. SolversLib 0.3.6, Pedro 3 and FTC SDK 12.

## Where to work

Open `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`:

```text
Robot.java       Hardware ownership and read → commands → write lifecycle
Constants.java   Alliance and Pedro drivetrain/Pinpoint configuration
config/          Vision, mechanism and measured-shot settings
subsystems/      Drive, Shooter, Turret, Flywheel, Hood, Intake, Kickup and Indexer
opmodes/         Competition TeleOp and guided tuning modes
  autos/         Pedro paths, reusable actions and competition auto modes
lib/             Reusable control, vision, localization, math and Pedro support
```

Normal robot changes belong in `config`, `subsystems`, or `opmodes`. Library internals stay out of the top-level workflow. See the [code map](docs/CODE_STRUCTURE.md).

## Build and operate

Use Android Studio's bundled JBR and run:

```text
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest
```

On Windows use `gradlew.bat`. Keep the pinned Gradle/AGP versions.

- [Empirical shot maps and optional physics correction](docs/PHYSICS_SHOOTING.md)
- [AdvantageScope logging and connection guide](docs/LOGGING.md)
- [Code-page copy: vision, aiming and logging](docs/CODE_PAGES.md)
- [Controls and calibration](docs/ROBOT_INTEGRATION.md)
- [Validation and remaining hardware checks](docs/VALIDATION.md)
- [Official field geometry](docs/FIELD_GEOMETRY.md)
- [Seattle Solvers audit](docs/SEATTLE_DECODE_REVIEW.md)
- [FTC SDK documentation and release history](docs/FTC_SDK_README.md)

Calibration flags start false. Desktop tests do not establish physical robot readiness.
