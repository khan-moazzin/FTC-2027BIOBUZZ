# BIOBUZZ 2026–27 — Project Notes

Background for `CLAUDE.md`. Read when you need the *why* behind a decision, the
verified API details, or the environment history. Not needed every session.

---

## Decision log

**Architecture went state-machine → Ivy command-based (2026-09-16).** The season
started aiming to reuse MiniRex's 2025–26 state-machine architecture
(github.com/khan-moazzin/FTC2026) for speed. That was reversed deliberately in
favor of Ivy. Last season's naming, formatting and file layout were kept.

**`Constants` moved out of `pedro/` into `org.firstinspires.ftc.teamcode`.** The
Pedro Quickstart ships `pedro/Constants.java`; this repo's copy had diverged and
was referenced by `Tuning.java` and the examples. It was moved via Refactor →
Move Class (not deleted) so all references updated automatically. Two classes
named `Constants` in one project is how someone edits the wrong file at a
competition.

**Config variable names match tuner output verbatim.** `localizerConfig`,
`drivetrainConfig`, `foresightConfig` — chosen so generated blocks paste in with
zero renaming. `mecanumConfig` was renamed to `drivetrainConfig` for exactly
this reason.

**`createLocalizer` / `createDrivetrain` are split out of `createFollower`** so
`Tuning.java` passes `Constants::createLocalizer, Constants::createDrivetrain`
to ForesightTuner. This guarantees the tuner measures the same configuration the
robot drives, instead of tuning one setup and running another.

**`Constants.create` → `createFollower`.** Matches last season's naming. The
Pedro example OpModes call `Constants.create(...)` and need updating.

**`Drivetrain` → `Drive`** (user preference).

**Intake and indexer are one motor.** Started as two; simplified to one motor
driving both, so they always turn together.

**Shoot-on-the-move is deferred, not adopted.** FRC 5817's 2026 robot has a full
SOTM solver. FTC robots move far slower, so motion compensation buys much less
while adding the frame-conversion bug class that cost 5817 a full debugging
cycle. Default plan: shoot from rest with an interpolated map, add compensation
only if match data demands it.

---

## Lessons from last season's code — do not repeat

From reviewing github.com/khan-moazzin/FTC2026:

1. **Two Followers on one HardwareMap.** `A6.java:44` called `mRobot.init()`
   (which built a Follower) then `A6.java:47` built a second one. Line 71 updated
   one, line 73 the other — two controllers writing the same four motors every
   loop. `Leave.java` did it right with `follower = mRobot.drive`.

2. **A P-controller whose setpoint was never used.** TeleopMain declared
   `TARGET_DISTANCE_IN = 12.0` at line 20 and never referenced it; line 92 was
   `forward = distance * FORWARD_kP`. Setpoint was effectively zero, so auto-align
   drove *into* the AprilTag instead of holding 12 inches.

3. **One error signal feeding two axes.** The same `tx` drove both strafe
   (line 90) and rotate (line 94). The corrections fight each other.

4. **`Math.abs()` hiding a sign error.** `getForwardDistanceInches()` wrapped a
   fixed-geometry `(camH - tagH)/tan(pitch + ty)` in `Math.abs()`, which also
   blows up as the angle crosses zero. The code comment literally said "feels
   WRONG."

5. **Blocking spin in `init()`.** `while (catapult.state == RETURNING)` ran a
   mechanism under power before START, with no timeout. Init sequencing belongs
   in `init_loop()`.

6. **Two sources of heading truth.** A degrees-valued `imuOffset` applied to a
   negated radian IMU read, then negated again at the call site — a load-bearing
   double negation, duplicating heading the localizer already owned. This year
   the Pinpoint via the localizer is the only source.

7. **Vision fused at a fixed weight with no rejection gate.** `FusedLocalizer`
   lerped vision in at 0.08 with no tag-count, ambiguity, distance or
   disagreement check, so one bad botpose yanked the pose. It was abandoned
   mid-season. Pedro 3.0 ships `FusionLocalizer` — a real timestamped Kalman
   filter with covariance and a history buffer for late measurements. Use it and
   tune the variances.

8. **Copy-pasted alliance mirrors that drifted.** `A6Mirror`/`A6PathMirror` were
   whole-file duplicates and had already silently diverged (Path2 used y=83.8697
   in one and y=81.8697 in the other). Parameterize mirroring with
   `PoseFactory.mirrorX(72)`.

9. **A README documenting packages that did not exist** (`/states`, `/controls`).

---

## Verified API facts

Checked against source, not assumed.

### Pedro Pathing 3.0 is not source-compatible with 2.0.3

- `Pose` moved: `com.pedropathing.geometry.Pose` → **`com.pedropathing.math.Pose`**
- Localizers now in **`com.pedropathing.revhub.localizers`**
- `MecanumConstants` → **`com.pedropathing.revhub.drivetrains.MecanumConfig`**
- **`FollowerBuilder` no longer exists.** Constructor is
  `Follower(Localizer, Drivetrain, Algorithm)` — note the Quickstart's stub
  comments it as `new Follower(Drivetrain, Localizer, Foresight)`, which has the
  first two backwards. Trust the signature.
- Configuration is a `ConfigVar` + lambda pattern, not a fluent builder
- `follow(Path)` replaces `followPath(PathChain)`; `manual(...)` replaces
  `setTeleOpDrive(...)`
- New types: `MotionState`, `Velocity`, `Twist`

`ForesightConfig` has **twelve required fields** with no library defaults — five
controllers, three brake-coefficient matrices, four kinematic limits. Unset
required fields throw `IllegalStateException: Config variable has not been set`
at *runtime*, not compile time. The Quickstart ships `Constants.java` as
`return null;` because all of it is expected to come from AutoTune.

### Ivy

- No `Subsystem` class — requirements are `Set<Object>`, so `.requiring(this)`
- No trigger/binding layer — edge-detect buttons in the OpMode loop
- No default commands — `InterruptedBehavior.SUSPEND` is the substitute; the
  Scheduler auto-resumes suspended commands when requirements free up
- No periodic hook — `Scheduler.execute()` only runs scheduled commands
- Group commands aggregate child requirements and take max child priority, so
  `.until(...)` preserves the wrapped command's requirement
- `Scheduler` is static and survives OpMode restarts — `Scheduler.reset()` in
  `init()` is mandatory

### Hardware

Axon MAX MK2: 500–2500µs PWM over 360°, which is exactly FTC's default `Servo`
range. Plain `setPosition()` gets full travel — no `ServoImplEx`/`setPwmRange`.
The servos close their own position loop, so there is no turret PID in our code.
Axon servos do have a fourth-wire analog position output, but Axon's docs for it
say "coming soon" and it is not wired or used here.

`DcMotorEx.setVelocity()` is persistent — the hub holds it. The flywheel needs no
per-loop update.

---

## Environment history

**Android Studio.** The project needs Quail 4 (2026.1.4) or newer. Ladybug
(2024.2.x) fails with "incompatible AGP 8.13.2, latest supported 8.7.0" because
it tops out near AGP 8.7. FIRST's admin confirmed Quail compatibility on the FTC
community forum in June 2026.

**Sloth resolution failure.** `Failed to resolve:
dev.frozenmilk.sinister:Sloth:0.3.0` with a suspiciously fast "BUILD SUCCESSFUL
in 1s" means Gradle replayed a *cached* resolution failure, not a fresh attempt.
Fix with `./gradlew --refresh-dependencies`. The artifact exists and the host is
up; the root cause was network. Note a Maven repo root returns 404 in a browser
by design — that proves nothing. Test with a real artifact path like
`https://repo.dairy.foundation/releases/dev/frozenmilk/sinister/Sloth/`.

**Terminal Gradle.** `Unable to locate a Java Runtime` means the shell can't see
Android Studio's bundled JDK. Do **not** install Java from java.com. Set:
`export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"`

**Network.** School/district wifi has blocked `repo.dairy.foundation` and killed
the TLS handshake to `claude.ai` (`SSL_ERROR_SYSCALL`). A phone hotspot or VPN
gets around it. Clean Gradle builds need `repo.dairy.foundation` and Maven
Central every time.

**Android Studio prompts to decline:** the AGP upgrade (pushes past the pinned
8.13.2) and "migrate to Daemon toolchain" (writes a JVM-pinning file FIRST does
not ship). Files at the repo root that aren't Gradle scripts are invisible in
the Project pane's **Android** view — switch the dropdown to **Project** to see
`CLAUDE.md`.

---

## Porting from FRC 2026 (Team 5817)

Source: github.com/5817Programming/2026Code

Worth bringing over: the **interpolated shot map** (distance → hood angle +
flywheel RPM), **turret aiming off field pose**, and a **`readyToShoot` gate**
before the indexer feeds.

Hard-won lessons from that codebase:

- **NaN/infinity guards on every solver output are mandatory.** An unguarded NaN
  out of `ShootingPlanner.recommendedShooterState()` caused an AdvantageKit
  logging BufferOverflowException.
- Field-relative velocities applied in the robot frame, `getChassisSpeeds()`
  returning robot-relative speeds, and a backwards world-to-robot turret demand
  conversion were three separate root-cause bugs in one SOTM system. Frame
  conversions are where this class of code goes wrong.
- Linear drag compensation used effective TOF `= (1 - e^(-tof·k))/k` with
  `DRAG_K = 0.375` for 2026 fuel balls. Not directly transferable to FTC game
  pieces, but the shape of the model is.
- A vision logging path caused an NT4 buffer overflow that lost a match
  connection. Log deliberately, not exhaustively.