package org.firstinspires.ftc.teamcode.config;

/** Coefficients require system-tuner measurements; zero gains are NOT calibrated. */
public final class MechanismConfig {
  public MechanismConfig() {}

  public static boolean flywheelCalibrated = false, hoodCalibrated = false;
  // False preserves old duty-cycle gains. Tune 3 exports true and fits volts, volts/RPM.
  public static boolean flywheelGainsInVolts = false;
  public static double ticksPerRev = 28, maxRpm = 6000;
  public static double[] flywheelP = {0, 0}, flywheelS = {0, 0}, flywheelV = {0, 0};
  public static double flywheelTolerance = 75, readySeconds = .15;
  public static double hoodMin = .15,
      hoodMax = .85,
      hoodStow = .15,
      hoodSettle = .25,
      hoodZero = 0,
      hoodRadiansPerUnit = 1;
  public static double turretTolerance = Math.toRadians(3), assistP = 1, assistMax = .35;
}
