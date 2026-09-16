# BIOBUZZ 2026–27 — FTC Team 26282

FTC DECODE-successor season code. Mecanum drivebase, Pinpoint odometry, turret
shooter. Written by one student programmer who also codes FRC Team 5817.

## Ground rules

- **Correctness over speed.** This code runs on a competition robot. Verify API
  signatures against source rather than assuming them.
- **Minimal, targeted changes.** No new abstractions unless the task requires it.
- **Never invent tuning numbers.** If a value must come from measurement or a
  tuner, mark it as a placeholder and say so. Do not write a plausible-looking
  constant and leave it unmarked.
- **Push back.** If something is poorly designed, inconsistent, or risky, say so
  directly instead of implementing it silently.

## Build environment

- Android Studio Quail 4 (2026.1.4) or newer. Ladybug **cannot** open this project.
- AGP 8.13.2 / Gradle 9.1.0, FTC SDK 12.0.0, Pedro Pathing 3.0, Ivy 1.1.1.
- Terminal Gradle needs `JAVA_HOME` pointed at Studio's bundled JBR:
  `/Applications/Android Studio.app/Contents/jbr/Contents/Home`
- `dev.frozenmilk.sinister:Sloth` arrives transitively from Pedro and resolves
  **only** from `https://repo.dairy.foundation/releases/`. That line must stay in
  `build.dependencies.gradle`. Never pin its version by hand.
- No Panels or FtcDashboard in dependencies — no `@Configurable`, no live
  constant tuning, plain FTC telemetry only.
- Decline Android Studio's AGP upgrade prompt and its "migrate to Daemon
  toolchain" prompt. Both break a pinned FTC build.

## Architecture: Ivy command-based

`com.pedropathing.ivy:pedro:1.1.1`. Facts verified against Ivy source:

- **No `Subsystem` class.** `Command.requirements()` is a `Set<Object>`, so the
  subsystem instance is the requirement token: `.requiring(this)`.
- **No trigger/binding layer.** Buttons are bound in the OpMode loop with
  explicit rising-edge detection.
- **No default commands.** The replacement is
  `InterruptedBehavior.SUSPEND` — the Scheduler auto-resumes a suspended command
  once its requirements free up. `Drive.teleopDrive()` uses this so a path-follow
  or auto-align command can take the drivetrain and driver control returns on its
  own. Without it the driver loses the sticks for the rest of the match.
- **No periodic hook.** `Scheduler.execute()` only runs scheduled commands.
  Per-loop hardware I/O is called explicitly from `Robot.update()`.
- Group commands aggregate child requirements and take max child priority, so
  `.until(...)` preserves the wrapped command's requirement.

### Subsystem pattern

Plain class owning hardware. Public API is **command factory methods**:

```java
public Command intake() {
    return Command.build()
            .setStart(() -> motor.setPower(INTAKE))
            .setEnd(end -> motor.setPower(INTAKE_IDLE))
            .requiring(this);
}
```

`setEnd` must make the mechanism safe on every exit path — natural, interrupted
and suspended all route through it. Leave `done()` at its default `false` for
hold-to-run mechanisms and bound them at the call site with `.until(...)`.

### OpMode pattern

```
init():  Scheduler.reset(); build Robot; set initial orientation
start(): Scheduler.schedule(drive.teleopDrive(...))
loop():  edge-detect buttons -> Scheduler.schedule(...)
         Scheduler.execute();   // commands compute desired powers
         mRobot.update();       // pushes them to hardware
stop():  Scheduler.reset();
```

`Scheduler.reset()` in `init()` is mandatory — the Scheduler is static state that
survives OpMode restarts. Loop order is load-bearing: reversing
`Scheduler.execute()` and `mRobot.update()` applies last loop's powers.

## Layout

```
teamcode/
    Constants.java     subsystem values + Pedro tuner config blocks
    Robot.java         container; update() = drive.update() + telemetry
    subsystems/        Drive, Intake, Turret, Hood, Flywheel
    OpModes/           TeleopMain
    pedro/             Tuning.java, examples/, procedures/  (Pedro's, untouched)
```

Only one `Constants`, in `org.firstinspires.ftc.teamcode`. Config variable names
match what the tuners generate verbatim — `localizerConfig`, `drivetrainConfig`,
`foresightConfig` — so tuner output pastes in with no renaming.
`createLocalizer(hw)` / `createDrivetrain(hw)` exist so `Tuning.java` can pass
`Constants::createLocalizer, Constants::createDrivetrain` to ForesightTuner,
which guarantees the tuner measures the config the robot actually drives.

An OpMode never constructs its own `Follower`. `Drive` builds it, once.

## Hardware

Driver Hub configuration names:
`fl` `bl` `fr` `br` `pinpoint` `intake` `turret1` `turret2` `hood`
`flywheel1` `flywheel2`

- Odometry: goBILDA Pinpoint + two **SWYFT** linear pods. SWYFT are third-party,
  so PinpointTuner must be run with pod type **CUSTOM**. `PinpointLocalizer` only
  applies `podType` when `ticksPerUnit` is empty, and `podType` defaults to
  `goBILDA_4_BAR_POD` — leaving `ticksPerUnit` unset silently applies goBILDA's
  resolution to SWYFT hardware and every distance is wrong with no error.
- Intake: one motor driving roller and indexer together.
- Turret: two Axon MAX MK2 servos. Hood: one. Axon MAX is 500–2500µs over 360°,
  exactly FTC's default `Servo` PWM range, so plain `setPosition()` gets full
  travel — no `ServoImplEx`/`setPwmRange`. The servos close their own loop, so
  there is no turret PID in our code.
- Flywheel: two goBILDA motors, hub velocity control. `setVelocity()` is
  persistent, so there is no per-loop update.

`turret2` and `flywheel2` are set REVERSE, correct only if each pair is mounted
facing each other. Same-direction mounting means they fight and stall.

## Drive sign convention

Pedro's mecanum mixing is `fl = axial - lateral - yaw`, `fr = axial + lateral +
yaw`, `bl = axial + lateral - yaw`, `br = axial - lateral + yaw`. So `+lateral`
is LEFT and `+yaw` is COUNTERCLOCKWISE. Gamepad sticks are +right and +down, so
**all three axes are negated** in the OpMode. Pedro's own `Pedro3TeleOp` example
negates none of them and drives inverted — do not copy it.

## Tuning order

1. MecanumTuner → paste `drivetrainConfig`
2. PinpointTuner, pod type CUSTOM → paste `localizerConfig`
3. ForesightTuner → paste `foresightConfig`

`ForesightConfig` has twelve required fields with no library defaults. Everything
currently in those blocks is a conservative placeholder so an untuned robot
crawls. Replace generated blocks wholesale; do not hand-edit individual numbers.

## Open questions

- `FLYWHEEL_TICKS_PER_REV = 28.0` assumes a bare 1:1 goBILDA motor. If the
  flywheel motors are geared this is wrong by the gear ratio and every RPM number
  is off. Needs the part number.
- Turret and hood travel limits (0.15 / 0.85) are placeholders; measure on the
  robot.
- Flywheel velocity PIDF is deliberately at hub defaults — tune it with the
  shooter work, do not invent values.

## Not yet built

Shooter logic (interpolated distance → hood angle + flywheel RPM map, turret
aiming off field pose, `readyToShoot` gate), Limelight vision, autos.

When shooter work starts, port from FRC Team 5817's 2026 code
(https://github.com/5817Programming/2026Code): the interpolated shot map, turret
aiming, and the readyToShoot gate. NaN/infinity guards on every solver output are
mandatory — that code threw a logging BufferOverflow from an unguarded NaN.
Full shoot-on-the-move is an open question, not a decision: FTC robots move far
slower, so motion compensation buys little while adding a whole class of
frame-conversion bugs. Default to shooting from rest with an interpolated map.

## How to verify changes without a robot

Pedro `core`, Ivy `core` and Ivy `pedro` are pure Java with no external
dependencies, so they can be cloned and compiled for real:

```
git clone --depth 1 https://github.com/Pedro-Pathing/PedroPathing.git
git clone --depth 1 https://github.com/Pedro-Pathing/Ivy.git
```

Stub only the FTC SDK types (`DcMotor`, `DcMotorEx`, `Servo`, `HardwareMap`,
`OpMode`, `Gamepad`, `ElapsedTime`, `Telemetry`) and Pedro's `revhub` classes
(`Mecanum`, `MecanumConfig`, `PinpointLocalizer`, `PinpointConfig`). Then compile
team code against them, call `Constants.createFollower()` to force every
`ConfigVar.required()` to be read — unset required fields throw
`IllegalStateException: Config variable has not been set` at runtime, not compile
time — and drive `Scheduler.execute()` with fake motors to verify command
lifecycles.