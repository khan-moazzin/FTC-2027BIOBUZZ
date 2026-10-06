package org.firstinspires.ftc.teamcode.lib.control;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.config.PhysicsShotConfig;

/** Shared generator/runtime normalization and physical-configuration fingerprint. */
public final class ShotPolynomial {
  public static double[] basis(double distance, double radial, PhysicsShotConfig c) {
    double d = (2 * distance - c.maxDistance - c.minDistance) / (c.maxDistance - c.minDistance);
    double v = radial / c.maxRadialSpeed;
    return new double[] {1, d, v, d * d, d * v, v * v};
  }

  public static double evaluate(double[] coefficients, double[] basis) {
    if (coefficients == null || coefficients.length != 6) return Double.NaN;
    double v = 0;
    for (int i = 0; i < 6; i++) v += coefficients[i] * basis[i];
    return v;
  }

  public static String signature(PhysicsShotConfig c) {
    return Arrays.toString(
            new double[] {
              c.speedPerRpm,
              c.launchAngleOffset,
              c.dragPerMeter,
              c.speedSigmaFraction,
              c.angleSigma,
              c.errorMultiplier,
              c.openingRadius,
              c.projectileRadius,
              c.clearanceMargin,
              c.muzzleHeight,
              c.targetHeight,
              c.minDistance,
              c.maxDistance,
              c.maxRadialSpeed,
              c.minLaunchAngle,
              c.maxLaunchAngle,
              c.maxExitSpeed,
              c.maxFlightSeconds,
              c.modelMaxMiss,
              c.heightTolerance
            })
        + Arrays.toString(c.apertureOffset)
        + Arrays.toString(c.apertureNormal)
        + Arrays.toString(
            new double[] {
              MechanismConfig.hoodZero, MechanismConfig.hoodRadiansPerUnit, MechanismConfig.maxRpm
            });
  }

  public static boolean physicalInputsValid(PhysicsShotConfig c) {
    double[] positive = {
      c.speedPerRpm,
      c.openingRadius,
      c.projectileRadius,
      c.minDistance,
      c.maxDistance,
      c.maxRadialSpeed,
      c.maxExitSpeed,
      c.maxFlightSeconds,
      c.modelMaxMiss,
      c.heightTolerance
    };
    for (double x : positive) if (!Double.isFinite(x) || x <= 0) return false;
    double[] nonnegative = {
      c.dragPerMeter,
      c.speedSigmaFraction,
      c.angleSigma,
      c.clearanceMargin,
      c.errorMultiplier,
      c.muzzleHeight
    };
    for (double x : nonnegative) if (!Double.isFinite(x) || x < 0) return false;
    if (c.apertureOffset == null
        || c.apertureNormal == null
        || c.apertureOffset.length != 3
        || c.apertureNormal.length != 3) return false;
    for (double x : c.apertureOffset) if (!Double.isFinite(x)) return false;
    double n = 0;
    for (double x : c.apertureNormal) {
      if (!Double.isFinite(x)) return false;
      n += x * x;
    }
    return Math.abs(n - 1) < 1e-6
        && Double.isFinite(c.targetHeight)
        && Double.isFinite(c.launchAngleOffset)
        && c.maxDistance > c.minDistance
        && c.maxFlightSeconds <= 3
        && Double.isFinite(c.minLaunchAngle)
        && Double.isFinite(c.maxLaunchAngle)
        && c.minLaunchAngle > 0
        && c.maxLaunchAngle < Math.PI / 2
        && c.maxLaunchAngle > c.minLaunchAngle
        && c.openingRadius > c.projectileRadius + c.clearanceMargin;
  }
}
