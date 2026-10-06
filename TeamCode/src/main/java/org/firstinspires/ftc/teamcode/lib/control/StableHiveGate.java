package org.firstinspires.ftc.teamcode.lib.control;

import org.firstinspires.ftc.teamcode.config.PhysicsShotConfig;
import org.firstinspires.ftc.teamcode.lib.field.Field;

/** Dwell requires distinct accepted vision timestamps, never repeated reads of one frame. */
public final class StableHiveGate {
  private long first = -1, last = -1;
  private int frames, endpoint;

  public boolean update(
      double angle, double rate, boolean fresh, long capture, long now, PhysicsShotConfig c) {
    int side = angle >= 0 ? 1 : -1;
    boolean eligible =
        fresh
            && capture <= now
            && capture >= 0
            && Double.isFinite(angle)
            && Double.isFinite(rate)
            && Double.isFinite(c.stableAngleTolerance)
            && c.stableAngleTolerance > 0
            && c.stableAngleTolerance < Field.MAX_ANGLE
            && Double.isFinite(c.stableRate)
            && c.stableRate >= 0
            && Double.isFinite(c.stableMaxFrameGap)
            && c.stableMaxFrameGap > 0
            && now - capture <= c.stableMaxFrameGap * 1e9
            && Double.isFinite(c.stableDwell)
            && c.stableDwell > 0
            && c.stableFrames >= 2
            && Math.abs(Math.abs(angle) - Field.MAX_ANGLE) <= c.stableAngleTolerance
            && Math.abs(rate) <= c.stableRate;
    if (!eligible
        || capture < last
        || side != endpoint
        || (last >= 0 && capture - last > c.stableMaxFrameGap * 1e9)) reset();
    if (!eligible) return false;
    endpoint = side;
    if (capture != last) {
      if (first < 0) first = capture;
      last = capture;
      frames++;
    }
    return frames >= c.stableFrames && last - first >= c.stableDwell * 1e9;
  }

  public void reset() {
    first = last = -1;
    frames = endpoint = 0;
  }
}
