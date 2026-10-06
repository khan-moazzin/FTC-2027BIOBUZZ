package org.firstinspires.ftc.teamcode.config;

/** Coefficients require system-tuner measurements; zero gains are NOT calibrated. */
public final class MechanismConfig {
  public MechanismConfig() {}

  public static boolean flywheelCalibrated = false, hoodCalibrated = false;
  // False preserves old duty-cycle gains. Tune 3 exports true and fits volts, volts/RPM.
  public static boolean flywheelGainsInVolts = false;
  public static double ticksPerRev = 28, maxRpm = 5800;
  public static double flywheelP = 0, flywheelS = 0, flywheelV = 0;
  public static double flywheelTolerance = 116, readySeconds = .15;
  public static double hoodMin = .15,
      hoodMax = .85,
      hoodStow = .15,
      hoodSettle = .25,
      hoodZero = .15,
      hoodRadiansPerUnit = 2 * Math.PI * 30.0 / 173.0;
  // Mechanical slope is known; Tune 4 must still measure the absolute launch-angle zero.
  public static double hoodSecondsPer60Degrees = .110, servoSettleMargin = .08;
  public static boolean indexerCalibrated = false;
  public static double indexerRetracted = .15,
      indexerDeployed = .85; // Main's placeholders; Tune 6.
  public static double shotReadySeconds = .10;
  public static boolean stationaryShotsOnly = false;
  public static double maxShotTranslation = 6, maxShotRotation = Math.toRadians(15);
  public static double turretMinRadians = Math.toRadians(-170),
      turretMaxRadians = Math.toRadians(170);
  public static double turretTolerance = Math.toRadians(3), assistP = 1, assistMax = .35;
}
