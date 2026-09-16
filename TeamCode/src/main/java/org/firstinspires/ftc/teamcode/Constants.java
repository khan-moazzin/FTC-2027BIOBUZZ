package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
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
 * Every value marked TUNER is a placeholder
 *   PinpointTuner  -> localizerConfig    (run with pod type CUSTOM, SWYFT pods)
 *   MecanumTuner   -> drivetrainConfig
 *   ForesightTuner -> foresightConfig
 */
public class Constants {

    // ================= INTAKE =================
    public static double INTAKE = 1.0;
    public static double OUTTAKE = -0.9;
    public static double INTAKE_IDLE = 0.0;

    // ================= FLYWHEEL =================
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;
    public static double FLYWHEEL_TOLERANCE = 75.0;

    // ================= TURRET =================
    // Servo units [0, 1]. TURRET_CENTER is the position that points the turret
    // straight forward, i.e. 0 degrees.
    public static double TURRET_MIN = 0.15;
    public static double TURRET_MAX = 0.85;
    public static double TURRET_CENTER = 0.5;

    /** Total travel between TURRET_MIN and TURRET_MAX. PLACEHOLDER: measure on the real turret.
     *  At center 0.5 that is -120 to +120. */
    public static double TURRET_RANGE_DEGREES = 240.0;

    // ================= HOOD =================
    public static double HOOD_MIN = 0.15;
    public static double HOOD_MAX = 0.85;
    public static double HOOD_STOW = 0.15;

    // ================= VISION =================
    // HIVE pivots, Pedro field inches. Taken from BiobuzzVision; verify against official CAD.
    public static double RED_HIVE_X = 59.25,  RED_HIVE_Y = 72.0;
    public static double BLUE_HIVE_X = 84.75, BLUE_HIVE_Y = 72.0;

    /** AprilTag pipeline on the Limelight. */
    public static int LIMELIGHT_PIPELINE = 1;

    // PLACEHOLDER: robot not CADed yet. Camera offset ahead of the turret axis, inches.
    public static double LL_FORWARD_FROM_TURRET_IN = 0.0;
    // PLACEHOLDER: robot not CADed yet. Turret axis from robot center, robot frame (+fwd, +left).
    public static double TURRET_FORWARD_IN = 0.0;
    public static double TURRET_LEFT_IN = 0.0;

    // ================= LOCALIZER =================
    // TUNER: PinpointTuner, pod type CUSTOM; podtypes

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.ticksPerUnit.set(OptionalDouble.of(1.0));                          // TUNER
        c.xPodOffset.set(0.0);                                               // TUNER
        c.yPodOffset.set(0.0);                                               // TUNER
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TUNER
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TUNER
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.INCH);
    });

    // ================= DRIVETRAIN =================
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
    // These placeholders are deliberately slow so the untuned robot crawls.
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(
                Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.1)));
        c.strafeTranslational.set(
                Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.1)));

        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.01, 0.01));

        c.linearBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));

        c.maxAchievableForwardVelocity.set(20.0);
        c.maxAchievableStrafeVelocity.set(15.0);
        c.naturalForwardDeceleration.set(30.0);
        c.naturalStrafeDeceleration.set(25.0);
    });

    // -----------------------------------------------------
    // FOLLOWER FACTORY
    // -----------------------------------------------------
    public static Follower createFollower(HardwareMap hw) {
        return createFollower(hw, createLocalizer(hw));
    }

    /** Overload so Drive can keep a typed handle for addMeasurement(). */
    public static Follower createFollower(HardwareMap hw, Localizer localizer) {
        return new Follower(localizer, createDrivetrain(hw), new Foresight(foresightConfig));
    }

    /** Pinpoint wrapped in Pedro's Kalman filter so Limelight frames fuse in by timestamp. */
    public static FusionLocalizer createLocalizer(HardwareMap hw) {
        return new FusionLocalizer(
                new PinpointLocalizer(hw, localizerConfig),
                new Pose(1.0, 1.0, Math.toRadians(5)),     // initial covariance
                new Pose(0.05, 0.05, Math.toRadians(0.5)), // process variance per update
                new Pose(2.0, 2.0, Math.toRadians(10)),    // measurement variance: TUNER, trust odom first
                50);
    }

    public static Drivetrain createDrivetrain(HardwareMap hw) {
        return new Mecanum(hw, drivetrainConfig);
    }
}