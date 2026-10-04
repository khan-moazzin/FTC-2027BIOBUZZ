package org.firstinspires.ftc.teamcode.config;

/** Replace with wizard export. Units: inches, radians, seconds, volts. */
public class VisionConfig {
  public boolean calibrated = false, turretCalibrated = false;
  public String camera = "limelight",
      leftFeedback = "turretEncoder1",
      rightFeedback = "turretEncoder2";
  public int pipeline = 1;
  public double leftZero = 0, rightZero = 0, leftSign = 1, rightSign = -1, analogRange = 3.3;
  public double servoCenter = .5, radiansPerServo = 2 * Math.PI, servoMin = .5, servoMax = .5;
  public double feedbackTolerance = Math.toRadians(5), turretLag = 0, maxTurretRate = 1;
  public double[] robotToTurret = {0, 0, 0}, turretToCamera = {0, 0, 0}, cameraRotation = {0, 0, 0};
  public double extraLatency = 0, maxFrameAge = .25, hiveMaxAge = .4;
  public double positionSigma = 2,
      angleSigma = Math.toRadians(5),
      hiveAccelerationNoise = 2,
      exposureSeconds = .01;
  public double maxFitRms = 3, innovationGate = 11.345, minRange = 4, maxRange = 120;
  public double maxPoseSigma = 4, maxHiveSigma = Math.toRadians(8);
}
