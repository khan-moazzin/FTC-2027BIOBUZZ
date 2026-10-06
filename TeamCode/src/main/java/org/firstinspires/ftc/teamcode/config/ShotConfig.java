package org.firstinspires.ftc.teamcode.config;

/** Measured successful shots only; wizard export replaces this file. */
public class ShotConfig {
  public boolean calibrated = false;

  /**
   * Main bea78b3 trial: distance inches, hood degrees above lowest position, flywheel percent.
   * Preserved separately: target height and flight time were not recorded.
   */
  public double[] referenceTrial = {48.0, 10.61, 62.1};

  public double transferDelay = 0.15;
  public double[] turretToMuzzle = {0, 0, 0};

  /** Each row: horizontal inches, height difference inches, RPM, hood radians, flight seconds. */
  public double[][] samples = {};
}
