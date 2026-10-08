package org.firstinspires.ftc.teamcode.opmodes.autos.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.Constants;

/** Pedro Path Generator route. Blue is the authored side; red mirrors across field center X. */
public final class ThreeTipPath {
  public final Pose start;
  public final Path intake1,
      intake2Prep,
      intake2Flower,
      intake3Prep,
      intake3,
      shoot1,
      intake4Flower,
      intake4,
      shoot2,
      park;

  public ThreeTipPath(Constants.Alliance alliance) {
    PoseFactory poses = PoseFactory.degrees();
    if (alliance == Constants.Alliance.RED) poses = poses.mirrorX(72);

    start = poses.of(132, 125.6262, 90);
    Pose intake1Pose = poses.of(132, 132, 90);
    Pose intake2PrepPose = poses.of(94.5, 21.4, -90);
    Pose intake2PrepControl1 = poses.of(111.1098, 97.828, 0);
    Pose intake2PrepControl2 = poses.of(122.9552, 26.2697, 0);
    Pose intake2FlowerPose = poses.of(94.5, 14, -90);
    Pose intake3PrepPose = poses.of(124, 36, 0);
    Pose intake3PrepControl1 = poses.of(91.4143, 23.1325, 0);
    Pose intake3Pose = poses.of(132, 36, 0);
    Pose shoot1Pose = poses.of(106.5512, 122.9743, 0);
    Pose intake4FlowerPose = poses.of(120, 94.5, 0);
    Pose intake4Pose = poses.of(127.9, 94.5, 0);
    Pose shoot2Pose = poses.of(108.6349, 123.5734, 0);
    Pose shoot2Control1 = poses.of(109.582, 86.9837, 0);
    Pose parkPose = poses.of(128.5244, 48.5305, 0);

    intake1 = line(start, intake1Pose).constant(intake1Pose);
    intake2Prep =
        curve(intake1Pose, intake2PrepControl1, intake2PrepControl2, intake2PrepPose)
            .linear(intake1Pose, intake2PrepPose);
    intake2Flower = line(intake2PrepPose, intake2FlowerPose).tangent();
    intake3Prep =
        curve(intake2FlowerPose, intake3PrepControl1, intake3PrepPose)
            .linear(intake2FlowerPose, intake3PrepPose);
    intake3 = line(intake3PrepPose, intake3Pose).tangent();
    shoot1 = line(intake3Pose, shoot1Pose).constant(shoot1Pose);
    intake4Flower = line(shoot1Pose, intake4FlowerPose).constant(intake4FlowerPose);
    intake4 = line(intake4FlowerPose, intake4Pose).constant(intake4Pose);
    shoot2 = curve(intake4Pose, shoot2Control1, shoot2Pose).constant(shoot2Pose);
    park = line(shoot2Pose, parkPose).constant(parkPose);
  }
}
