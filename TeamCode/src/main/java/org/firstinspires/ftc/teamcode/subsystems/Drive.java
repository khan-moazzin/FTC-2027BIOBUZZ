package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.*;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.localization.BufferedFusionLocalizer;

public final class Drive extends SubsystemBase {
  private final BufferedFusionLocalizer localizer;
  private final Follower follower;
  private double headingOffset;
  private boolean robotOriented;

  public Drive(HardwareMap hw) {
    localizer = new BufferedFusionLocalizer(new PinpointLocalizer(hw, Constants.localizerConfig));
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
    if (robotOriented) follower.manual(x, y, yaw);
    else
      follower.manual(ManualDrive.fieldCentric(x, y, yaw, getPose().heading(), -driverForward()));
  }

  private double driverForward() {
    return Math.toRadians(Constants.driverForwardDegrees()) + headingOffset;
  }

  public void update() {
    follower.update();
  }

  public Follower getFollower() {
    return follower;
  }

  public Pose getPose() {
    return follower.pose();
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
