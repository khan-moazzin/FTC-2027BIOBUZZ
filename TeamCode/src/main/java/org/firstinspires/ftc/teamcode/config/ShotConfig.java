package org.firstinspires.ftc.teamcode.config;

/** Measured successful shots only; wizard export replaces this file. */
public class ShotConfig {
  public boolean calibrated = false;
  public double transferDelay = 0.15;
  public double[] turretToMuzzle = {0, 0, 0};

  /** Each row: horizontal inches, height difference inches, RPM, hood radians, flight seconds. */
  public double[][] samples = {};
}
