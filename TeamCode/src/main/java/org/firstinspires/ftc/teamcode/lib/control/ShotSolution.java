package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.lib.math.Vec3;

/** Shared mechanism targets and diagnostic values. Distances inches, angles radians. */
public class ShotSolution {
  public boolean valid, mapClamped, physicsCorrected;
  public String reason = "No solution";
  public Vec3 predictedTarget, muzzle, launchVelocity;
  public int iterations;
  public double impactVariance = Double.NaN, clearance = Double.NaN;
  public double angle, angularVelocity, rpm, hood, flight, distance, height;
}
