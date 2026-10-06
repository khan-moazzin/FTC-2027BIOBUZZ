package org.firstinspires.ftc.teamcode.config;

/** Measured successful shots and their independently interpolated mechanism maps. */
public class ShotConfig {
  public boolean calibrated = false;

  /**
   * Main bea78b3 trial: distance inches, hood degrees above lowest position, flywheel percent.
   * Preserved separately: target height and flight time were not recorded.
   */
  public double[] referenceTrial = {48.0, 10.61, 62.1};

  public double transferDelay = 0.15;
  public double[] turretToMuzzle = {0, 0, 0};

  /** Rows are {horizontal distance inches, hood launch angle degrees}. */
  public double[][] hoodMap = {{48.0, 10.61}};

  /** Rows are {horizontal distance inches, flywheel percent from 0 to 100}. */
  public double[][] flywheelMap = {{48.0, 62.1}};

  /** Physics-fit archive: horizontal inches, height difference, RPM, hood radians, flight seconds. */
  public double[][] samples = {};
}
