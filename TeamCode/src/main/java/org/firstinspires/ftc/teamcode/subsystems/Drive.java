package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.localization.BufferedFusionLocalizer;

public final class Drive extends SubsystemBase {
  private final BufferedFusionLocalizer localizer;
  private final Follower follower;
  public double commandForward, commandStrafe, commandYaw;
  private double headingOffset;
  private boolean robotOriented;

  public Drive(HardwareMap hw) {
    PinpointLocalizer raw = new PinpointLocalizer(hw, Constants.localizerConfig);
    GoBildaPinpointDriver pinpoint =
        hw.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    // getDeviceStatus reads the cached status from Pedro's one bulk update; no second I2C poll.
    localizer =
        new BufferedFusionLocalizer(
            raw, () -> pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
    follower = Constants.createFollower(hw, localizer);
  }

  public Command teleopDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier yaw) {
    return new RunCommand(() -> drive(x.getAsDouble(), y.getAsDouble(), yaw.getAsDouble()), this);
  }

  public Command follow(Path p) {
    FollowPathCommand c = new FollowPathCommand(follower, p, false);
    c.addRequirements(this);
    return c;
  }

  public void drive(double x, double y, double yaw) {
    commandForward = x;
    commandStrafe = y;
    commandYaw = yaw;
    if (robotOriented || !localizer.healthy(System.nanoTime())) follower.manual(x, y, yaw);
    else
      follower.manual(ManualDrive.fieldCentric(x, y, yaw, getPose().heading(), -driverForward()));
  }

  private double driverForward() {
    return Math.toRadians(Constants.driverForwardDegrees()) + headingOffset;
  }

  public void update() {
    if (!localizer.healthy(System.nanoTime()) && (follower.following() || follower.holding()))
      follower.stop();
    follower.update();
  }

  public Follower getFollower() {
    return follower;
  }

  public Pose getPose() {
    return localizer.pose();
  }

  public void setPose(Pose p) {
    follower.setPose(p);
  }

  public void resetHeading() {
    headingOffset = getPose().heading() - Math.toRadians(Constants.driverForwardDegrees());
  }

  public void toggleRobotOriented() {
    robotOriented = !robotOriented;
  }

  public boolean isRobotOriented() {
    return robotOriented;
  }

  public void stop() {
    commandForward = commandStrafe = commandYaw = 0;
    follower.stop();
    follower.update();
  }

  public BufferedFusionLocalizer localizer() {
    return localizer;
  }

  public void read(long now) {
    localizer.capture(now);
  }
}
