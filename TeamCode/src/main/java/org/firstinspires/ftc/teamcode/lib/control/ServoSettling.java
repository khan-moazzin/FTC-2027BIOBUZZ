package org.firstinspires.ftc.teamcode.lib.control;

/** Readiness starts at the actual output write and resets after cumulative target movement. */
public final class ServoSettling {
  private double reference = Double.NaN;
  private boolean pending = true, written;
  private long changed, travelReady;
  private double writtenReference = Double.NaN;

  public void command(double position) {
    if (!Double.isFinite(reference) || Math.abs(position - reference) > .002) {
      reference = position;
      pending = true;
    }
  }

  public void written(long now) {
    written(now, 0, 0);
  }

  public void written(long now, double secondsPerServoUnit, double margin) {
    if (pending || !written) {
      double distance =
          Double.isFinite(writtenReference) ? Math.abs(reference - writtenReference) : 1;
      double delay = distance * secondsPerServoUnit + margin;
      travelReady =
          Double.isFinite(delay) && delay >= 0
              ? Math.max(travelReady, now + (long) (delay * 1e9))
              : Long.MAX_VALUE;
      writtenReference = reference;
    }
    if (pending || !written) changed = now;
    pending = false;
    written = true;
  }

  public boolean ready(long now, double seconds) {
    return written
        && !pending
        && Double.isFinite(seconds)
        && seconds >= 0
        && now >= travelReady
        && now >= changed
        && now - changed >= seconds * 1e9;
  }
}
