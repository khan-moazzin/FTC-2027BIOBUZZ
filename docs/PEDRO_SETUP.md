# Pedro Pathing setup

This project retains FTC SDK 12.0.0 and adds `com.pedropathing:revhub:3.0.0`
and `com.pedropathing:tuning:1.0.0` from the Dairy Maven repository.
`revhub` includes the full `com.pedropathing:core:3.0.0` transitively.
`com.pedropathing.ivy:pedro:1.1.1` adds Ivy's command framework and its
Pedro 3 adapter (including Ivy core 1.1.1).

Pedro is a library used with the FTC SDK, not a replacement Robot Controller
SDK. This project uses the published Pedro 3 artifacts directly: the follower
returned by `Constants.create()` exposes the complete library API.

The `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro` package is
based on the [official quickstart](https://github.com/Pedro-Pathing/Quickstart/tree/b4312385b7d0cc5e8dd263ec3927c9ef0cb48f36),
revision `b4312385b7d0cc5e8dd263ec3927c9ef0cb48f36`. `Constants` and `Tuning`
are customized for a mecanum drivetrain with goBILDA Pinpoint. The upstream
procedures are included, including alternate localizers; only Mecanum,
Pinpoint, Foresight, and Tests are registered. The repository's FIRST license
also accompanies the imported quickstart code.

## Configure and tune

1. Open this repository in Android Studio and sync Gradle.
2. Match `Constants.drivetrainConfig` to the Robot Controller motor names.
   The starter names are `left_front`, `left_back`, `right_front`, and
   `right_back`. Left motors start reversed and right motors forward;
   confirm these with the Mecanum tuner.
3. Configure the Pinpoint device as `pinpoint`, or change
   `Constants.localizerConfig` to match its name.
4. Build and install TeamCode on the Robot Controller. While connected to
   its network, open [AutoTune](http://192.168.43.1:10158).
5. Run Mecanum Tuner, then Pinpoint Tuner. Select your actual odometry pod
   type and follow the measurement instructions. Replace the corresponding
   configuration blocks in `Constants.java` with the generated Java output.
   Pod offsets and directions are not calibrated by this repository.
6. Rebuild/install, run Foresight Tuner, and replace `foresightConfig` with
   its generated output. Rebuild/install again before using Tests.
7. Use the Tests procedure to verify localization, driving, and path following.

The starter settings and library defaults are not measurements of your robot.
Complete tuning before using the follower in autonomous. See the official
[tuning guide](https://pedropathing.com/docs/pathing/tuning).

## Use the follower

Create a follower in your OpMode's initialization:

```java
Follower follower = Constants.create(hardwareMap);
```

Import `com.pedropathing.follower.Follower` and
`org.firstinspires.ftc.teamcode.Constants`. Update the follower every
active loop with `follower.update()`. Follow the
[autonomous guide](https://pedropathing.com/docs/pathing/guide/setting-up-auto)
to define a starting pose and paths.

`BiobuzzVision.java` preserves the existing vision helper; its filename now
matches its public class. Tag ID and quality access uses FTC 12's single-tag
detection subtype; cluster detections are excluded from individual-tag fusion.
Its camera extrinsics and tag geometry still require
calibration as described in that class. Vision fusion is not enabled automatically.

## Pedro 3 features and examples

The examples live in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/examples`.
Remove `@Disabled` from the desired OpMode after tuning and reviewing its route.

| Feature | Where to start |
| --- | --- |
| Pose factories and mirroring | `DemoPaths`: degrees-based poses, optional `mirrorX(72)` |
| Lines, Bezier curves, composite paths | `DemoPaths.outbound` |
| Constant and linear heading interpolation | The individual outbound segments and home path |
| Per-path Foresight constraints | `maxVelocityConstraint.at(24.0)` in `DemoPaths` (inches/second) |
| Command sequences and waits | `Pedro3Auto`, using Ivy's scheduler and Pedro `follow` commands |
| Robot- and field-centric drive | `Pedro3TeleOp`; hold left bumper for robot-centric mode |
| Logging and follow state | Both OpModes attach `withLogger`; Auto also reports completion |
| Foresight and localizer tuning | Existing registered AutoTune procedures in `Tuning` |

The demonstration route starts at (36, 36, 0 degrees), follows a line and a
curve, pauses, and returns to its start. It is not a competition route.
The TeleOp establishes heading zero at initialization; it does not assume an
autonomous ending pose. Both OpModes stop the follower when stopped, and the
autonomous resets Ivy's static scheduler at initialization and shutdown.

Ivy is optional when writing your own OpModes: call `follower.follow(path)`
directly, call `follower.update()` every loop, and use `follower.following()`
to advance your autonomous. Use `follower.hold(pose)` to hold a position and
`follower.stop()` to stop it. When using Ivy, execute its scheduler once per
loop as shown in the example. Its groups also support parallel command work
for mechanisms while following paths.

Additional APIs are available directly without enabling another SDK:

- [Pose factory operations](https://pedropathing.com/docs/pathing/reference/pose-factory)
- [Pedro 3 features](https://pedropathing.com/docs/pathing/pedro3)
- [Logging](https://pedropathing.com/docs/pathing/reference/logging)
- [Ivy commands](https://pedropathing.com/docs/ivy/pedro-commands)
- [TeleOp and heading-based control](https://pedropathing.com/docs/pathing/guide/teleop-usage)

Use `Constants.foresightConfig` for controller, braking, path, and completion
settings. Per-path `.with(configVariable.at(value))` modifiers let individual
paths override settings without replacing the whole follower.

## Build

Use Android Studio's bundled JDK and the Android SDK configured in your local
`local.properties` (which remains machine-specific):

```powershell
$env:JAVA_HOME = 'C:/Program Files/Android/Android Studio/jbr'
./gradlew.bat :TeamCode:assembleDebug
```

Output: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
