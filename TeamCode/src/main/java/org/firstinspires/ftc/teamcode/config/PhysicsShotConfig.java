package org.firstinspires.ftc.teamcode.config;

/**
 * SI units. Measured calibration + offline-generated quadratic surfaces; no invented robot tune.
 */
public class PhysicsShotConfig {
  public boolean calibrated = false, modelGenerated = false, geometryVerified = false;
  public double speedPerRpm = 0, launchAngleOffset = 0, dragPerMeter = 0;
  public double speedSigmaFraction = .02, angleSigma = Math.toRadians(.5), errorMultiplier = 2;
  // Measure clear opening and projectile radii; clearance excludes an additional safety margin.
  public double openingRadius = 0, projectileRadius = 0, clearanceMargin = .01;
  // Offset of the actual aperture center from Field.cell reference, in the neutral HIVE frame (m).
  public double[] apertureOffset = {0, 0, 0};
  public double[] apertureNormal = {0, 0, 1};
  public double muzzleHeight = 0, targetHeight = 0;
  public double minDistance = .8, maxDistance = 3.0, maxRadialSpeed = .5;
  public double minLaunchAngle = Math.toRadians(20), maxLaunchAngle = Math.toRadians(75);
  public double maxExitSpeed = 12, maxFlightSeconds = 2;
  public double calibrationMaxError = .04, modelMaxMiss = .025, heightTolerance = .01;
  public double stableAngleTolerance = Math.toRadians(2), stableRate = Math.toRadians(2);
  public double stableDwell = .35, stableMaxFrameGap = .25;
  public int stableFrames = 3;
  // Normalized inputs d = (distance-midpoint)/halfRange; v = radialSpeed/maxRadialSpeed.
  // Terms: 1,d,v,d*d,d*v,v*v. Speed is relative to muzzle; angle is physical projectile angle.
  public double[] speedCoefficients = {}, angleCoefficients = {}, flightCoefficients = {};
  // Generator fingerprint invalidates polynomials when their physical inputs are changed.
  public String modelSignature = "";
}
