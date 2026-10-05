package org.firstinspires.ftc.teamcode.lib.control;

import java.util.function.DoubleSupplier;

/**
 * Monotonic 5 Hz polling. Invalid samples invalidate the cache instead of retaining old voltage.
 */
public final class CachedVoltage {
  private final DoubleSupplier sensor;
  private long sampledAt;
  private boolean sampled;
  private double volts = Double.NaN;

  public CachedVoltage(DoubleSupplier sensor) {
    this.sensor = sensor;
  }

  public void read(long now) {
    if (sampled && now >= sampledAt && now - sampledAt < 200000000L) return;
    sampledAt = now;
    sampled = true;
    double value = sensor.getAsDouble();
    volts = Double.isFinite(value) && value >= 6 && value <= 20 ? value : Double.NaN;
  }

  public double volts(long now) {
    return sampled && now >= sampledAt && now - sampledAt <= 500000000L ? volts : Double.NaN;
  }

  /** Gains expressed in volts are converted to duty cycle once, after feedback + feedforward. */
  public static double duty(double requestedVolts, double batteryVolts) {
    return Double.isFinite(requestedVolts)
            && Double.isFinite(batteryVolts)
            && batteryVolts >= 6
            && batteryVolts <= 20
        ? Angles.clamp(requestedVolts / batteryVolts, 0, 1)
        : 0;
  }
}
