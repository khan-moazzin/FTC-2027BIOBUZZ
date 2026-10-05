package org.firstinspires.ftc.teamcode.lib.vision;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.math.*;

public final class CameraGeometry {
  public static Vec3 toRobot(Vec3 camera, double turret, VisionConfig c) {
    return new Transform3(
            new Vec3(c.turretToCamera),
            c.cameraRotation[0],
            c.cameraRotation[1],
            c.cameraRotation[2])
        .apply(camera)
        .rotateZ(turret)
        .plus(new Vec3(c.robotToTurret));
  }

  public static Vec3 toField(Vec3 camera, double turret, Pose robot, VisionConfig c) {
    return toRobot(camera, turret, c)
        .rotateZ(robot.heading())
        .plus(new Vec3(robot.x(), robot.y(), 0));
  }

  public static Vec3 toCamera(Vec3 field, double turret, Pose robot, VisionConfig c) {
    Vec3 p =
        field
            .minus(new Vec3(robot.x(), robot.y(), 0))
            .rotateZ(-robot.heading())
            .minus(new Vec3(c.robotToTurret))
            .rotateZ(-turret);
    return new Transform3(
            new Vec3(c.turretToCamera),
            c.cameraRotation[0],
            c.cameraRotation[1],
            c.cameraRotation[2])
        .inverse(p);
  }
}
