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
    /** Encoder ticks per output revolution. goBILDA motor dependent. */
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    /** RPM error that still counts as up to speed. */
    public static double FLYWHEEL_TOLERANCE = 75.0;

    // ================= TURRET =================
    // Servo units [0, 1] over 360 degrees of travel.
    public static double TURRET_MIN = 0.15;
    public static double TURRET_MAX = 0.85;
    public static double TURRET_CENTER = 0.5;

    // ================= HOOD =================
    // Servo units [0, 1].
    public static double HOOD_MIN = 0.15;
    public static double HOOD_MAX = 0.85;
    public static double HOOD_STOW = 0.15;

    // ================= LOCALIZER =================
    // TUNER: PinpointTuner, pod type CUSTOM; podtypes
    // defaults to goBILDA_4_BAR_POD. Leaving ticksPerUnit unset
    // silently applies goBILDA's resolution and every distance is wrong.

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
        return new Follower(
                createLocalizer(hw),
                createDrivetrain(hw),
                new Foresight(foresightConfig)
        );
    }

    public static Localizer createLocalizer(HardwareMap hw) {
        return new PinpointLocalizer(hw, localizerConfig);
    }

    public static Drivetrain createDrivetrain(HardwareMap hw) {
        return new Mecanum(hw, drivetrainConfig);
    }
}