package org.firstinspires.ftc.teamcode.lib.control;

/** Readiness starts at the actual output write and resets after cumulative target movement. */
public final class ServoSettling {
  private double reference = Double.NaN;
  private boolean pending = true, written;
  private long changed;

  public void command(double position) {
    if (!Double.isFinite(reference) || Math.abs(position - reference) > .002) {
      reference = position;
      pending = true;
    }
  }

  public void written(long now) {
    if (pending || !written) changed = now;
    pending = false;
    written = true;
  }

  public boolean ready(long now, double seconds) {
    return written
        && !pending
        && Double.isFinite(seconds)
        && seconds >= 0
        && now >= changed
        && now - changed >= seconds * 1e9;
  }
}
