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
import java.util.OptionalDouble;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Every value marked TUNER is a placeholder PinpointTuner -> localizerConfig (run with pod type
 * CUSTOM, SWYFT pods) MecanumTuner -> drivetrainConfig ForesightTuner -> foresightConfig
 */
public class Constants {

  // ================= ALLIANCE =================
  public enum Alliance {
    RED,
    BLUE
  }

  /** Set in TeleopMain's init, or by an auto. Decides which way is "forward" for the driver. */
  public static Alliance ALLIANCE = Alliance.BLUE;

  // Placeholder in degrees. Verify on field
  public static double BLUE_DRIVER_FORWARD_DEG = 90.0;
  public static double RED_DRIVER_FORWARD_DEG = -90.0;

  public static double driverForwardDegrees() {
    return ALLIANCE == Alliance.RED ? RED_DRIVER_FORWARD_DEG : BLUE_DRIVER_FORWARD_DEG;
  }

  // ================= LOCALIZER =================
  // TUNER: PinpointTuner, pod type CUSTOM; podtypes

  public static PinpointConfig localizerConfig =
      new PinpointConfig(
          c -> {
            c.name.set("pinpoint");
            c.ticksPerUnit.set(OptionalDouble.of(1.0)); // TUNER
            c.xPodOffset.set(0.0); // TUNER
            c.yPodOffset.set(0.0); // TUNER
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TUNER
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD); // TUNER
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
            c.encoderResolutionUnit.set(DistanceUnit.INCH);
          });

  // ================= DRIVETRAIN =================
  public static MecanumConfig drivetrainConfig =
      new MecanumConfig(
          c -> {
            c.frontLeftName.set("fl");
            c.backLeftName.set("bl");
            c.frontRightName.set("fr");
            c.backRightName.set("br");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE); // TUNER
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE); // TUNER
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD); // TUNER
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD); // TUNER
          });

  // ================= PATH FOLLOWING =================
  // These placeholders are deliberately slow.
  public static ForesightConfig foresightConfig =
      new ForesightConfig(
          c -> {
            c.forwardTranslational.set(
                Controller.piecewise(Controller.proportional(0.1))
                    .put(2.5, Controller.proportional(0.1)));
            c.strafeTranslational.set(
                Controller.piecewise(Controller.proportional(0.1))
                    .put(2.5, Controller.proportional(0.1)));

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

  /**
   * Standalone Pedro tools update this raw localizer themselves. Competition Drive owns buffered
   * fusion.
   */
  public static Localizer createLocalizer(HardwareMap hw) {
    return new PinpointLocalizer(hw, localizerConfig);
  }

  public static Drivetrain createDrivetrain(HardwareMap hw) {
    return new Mecanum(hw, drivetrainConfig);
  }
}
