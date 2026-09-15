package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

/**
 * Central constants for the 2026-27 BIOBUZZ robot.
 *
 * Values are intentionally non-final public statics so Panels / FtcDashboard can
 * tune them live, same as last season.
 * Every value below marked "TUNER:" is a PLACEHOLDER. Pedro Pathing 3.0 is
 * built around its AutoTune procedures generating these for you, and there is
 * no correct value that can be written before the drivebase exists and has
 * been pushed around the field.
 *
 * The placeholders are deliberately conservative (slow) so an untuned robot
 * crawls instead of lurching. They are NOT correct and paths will not track
 * properly until each tuner has been run and its generated block pasted in.
 *
 * Tuner -> what it generates:
 *   PinpointTuner   -> localizerConfig   (pod offsets, directions, ticksPerUnit)
 *   MecanumTuner    -> mecanumConfig     (motor directions)
 *   ForesightTuner  -> foresightConfig   (all controllers + kinematics)
 * ===========================================================================
 */
public class Constants {

    // ================= INTAKE =================
    public static double INTAKE = 1.0;
    public static double OUTTAKE = -0.9;
    public static double INTAKE_IDLE = 0.0;

    // ================= FLYWHEEL =================
    /** Encoder ticks per output revolution of the flywheel motor. Motor-dependent. */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    /** How close to target RPM counts as ready to shoot. */
    public static double FLYWHEEL_RPM_TOLERANCE = 75.0;

    /** Velocity PIDF applied to the flywheel motor's built-in velocity controller. */
    public static double FLYWHEEL_kP = 10.0;
    public static double FLYWHEEL_kI = 0.0;
    public static double FLYWHEEL_kD = 0.0;
    public static double FLYWHEEL_kF = 14.0;

    // ================= HOOD =================
    /** Servo positions, in servo units [0, 1]. */
    public static double HOOD_MIN_POSITION = 0.15;
    public static double HOOD_MAX_POSITION = 0.85;
    public static double HOOD_STOW_POSITION = 0.15;

    // ================= TURRET =================
    /** Encoder ticks per radian at the turret output. Depends on gearing. */
    public static double TURRET_TICKS_PER_RADIAN = 400.0;

    /** Soft limits, radians, relative to the turret's zero (robot-forward). */
    public static double TURRET_MIN_RADIANS = Math.toRadians(-135);
    public static double TURRET_MAX_RADIANS = Math.toRadians(135);

    /** How close to the target angle counts as aimed. */
    public static double TURRET_ANGLE_TOLERANCE = Math.toRadians(1.5);

    public static double TURRET_kP = 1.2;
    public static double TURRET_MAX_POWER = 0.6;

    // ================= LOCALIZER =================
    // TUNER: PinpointTuner. Run it with Odometry Pod Type = CUSTOM.
    //
    // The SWYFT linear pods are NOT goBILDA pods. PinpointLocalizer only applies
    // podType when ticksPerUnit is empty, and podType defaults to
    // goBILDA_4_BAR_POD. Leaving ticksPerUnit unset silently applies goBILDA's
    // resolution to SWYFT hardware and every reported distance is wrong, with no
    // error. ticksPerUnit below MUST come from the tuner's CUSTOM path.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.ticksPerUnit.set(OptionalDouble.of(1.0));          // TUNER: placeholder
        c.xPodOffset.set(0.0);                                // TUNER: placeholder
        c.yPodOffset.set(0.0);                                // TUNER: placeholder
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);  // TUNER
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);  // TUNER
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
    });

    // ================= DRIVETRAIN =================
    // TUNER: MecanumTuner generates this whole block, under exactly this name.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fl");
        c.backLeftName.set("bl");
        c.frontRightName.set("fr");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);   // TUNER
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);    // TUNER
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);  // TUNER
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);   // TUNER
    });

    // ================= PATH FOLLOWING =================
    // TUNER: ForesightTuner generates this entire block. Replace it wholesale
    // with the generated code; do not hand-edit individual numbers.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.1);
        Controller primaryTranslationalLateral = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.01, 0.01));

        c.linearBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));

        // Conservative placeholders so an untuned robot crawls. Real values come
        // from ForesightTuner and will be substantially larger.
        c.maxAchievableForwardVelocity.set(20.0);
        c.maxAchievableStrafeVelocity.set(15.0);
        c.naturalForwardDeceleration.set(30.0);
        c.naturalStrafeDeceleration.set(25.0);
    });

    // -----------------------------------------------------
    // FOLLOWER FACTORY
    // -----------------------------------------------------
    /**
     * Single construction point for the Follower. Robot.init() calls this once;
     * no OpMode may ever build its own, or two controllers end up driving the
     * same four motors.
     */
    public static Follower createFollower(HardwareMap hw) {
        return new Follower(
                createLocalizer(hw),
                createDrivetrain(hw),
                new Foresight(foresightConfig)
        );
    }

    /**
     * Split out so the tuners can reuse them. ForesightTuner takes a
     * Function&lt;HardwareMap, Localizer&gt; and a Function&lt;HardwareMap, Drivetrain&gt;,
     * so Tuning.java registers it as:
     *
     *   new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain)
     *
     * That keeps the tuner and the robot running off one config, instead of the
     * tuner measuring one setup while the robot drives another.
     */
    public static Localizer createLocalizer(HardwareMap hw) {
        return new PinpointLocalizer(hw, localizerConfig);
    }

    public static Drivetrain createDrivetrain(HardwareMap hw) {
        return new Mecanum(hw, drivetrainConfig);
    }
}