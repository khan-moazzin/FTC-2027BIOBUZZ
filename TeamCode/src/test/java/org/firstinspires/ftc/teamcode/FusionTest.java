package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import com.pedropathing.localization.*;
import com.pedropathing.math.*;
import org.firstinspires.ftc.teamcode.localization.BufferedFusionLocalizer;
import org.junit.Test;

public class FusionTest {
  static class Odom implements Localizer {
    Pose p = Pose.zero();
    Velocity v = new Velocity(1, 0, 0);

    public void setPose(Pose p) {
      this.p = p;
    }

    public MotionState state() {
      return MotionState.ofVelocity(p, v);
    }

    public void update() {}

    public void reset() {
      p = Pose.zero();
    }
  }

  @Test
  public void faultClearsHistoryAndRequiresExplicitPoseRecovery() {
    Odom o = new Odom();
    boolean[] ready = {false};
    BufferedFusionLocalizer f = new BufferedFusionLocalizer(o, () -> ready[0]);
    f.capture(1000000000L);
    assertFalse(f.healthy(1000000000L));
    ready[0] = true;
    f.capture(1100000000L);
    assertTrue(f.healthy(1100000000L));
    assertFalse(f.healthy(1400000000L));
    ready[0] = false;
    f.capture(1200000000L);
    assertEquals(0, f.historySize());
    assertEquals(0, f.velocity().vx, 0);
    assertFalse(f.addMeasurement(Pose.zero(), 1200000000L, Matrix.diag(1, 1, 1), 12));
    ready[0] = true;
    o.p = new Pose(100, 0, 0); // Device may have reset its reference frame.
    f.capture(1300000000L);
    assertFalse(f.healthy(1300000000L));
    assertEquals(0, f.pose().x(), 0);
    f.setPose(new Pose(5, 6, 0));
    f.capture(1400000000L);
    assertTrue(f.healthy(1400000000L));
    assertEquals(5, f.pose().x(), 0);
  }

  @Test
  public void nonfiniteVelocityCannotEnterFusionOrRestoreReadiness() {
    Odom o = new Odom();
    BufferedFusionLocalizer f = new BufferedFusionLocalizer(o);
    f.capture(1000000000L);
    o.v = new Velocity(Double.NaN, 0, 0);
    f.capture(1100000000L);
    assertFalse(f.healthy(1100000000L));
    assertEquals(0, f.historySize());
    assertEquals(0, f.velocity().vx, 0);
  }

  @Test
  public void boundedHistoryResetAndInnovation() {
    Odom o = new Odom();
    BufferedFusionLocalizer f = new BufferedFusionLocalizer(o);
    long start = 1000000000L;
    for (int i = 0; i < 2000; i++) {
      o.p = new Pose(i * .01, 0, 0);
      long t = start + i * 1000000L;
      f.capture(t);
      if (i > 1) f.addMeasurement(o.p, t - 500000L, Matrix.diag(1, 1, .1), 12);
    }
    assertTrue(f.historySize() <= 512);
    assertFalse(
        f.addMeasurement(new Pose(1000, 1000, 0), start + 1999000000L, Matrix.diag(1, 1, .1), 12));
    f.setPose(new Pose(4, 5, 1));
    assertEquals(0, f.historySize());
    assertEquals(4, f.pose().x(), 0);
    f.reset();
    assertEquals(0, f.pose().x(), 0);
  }

  @Test
  public void delayedCorrectionPropagatesAndOldFrameCannotEraseIt() {
    Odom o = new Odom();
    BufferedFusionLocalizer f = new BufferedFusionLocalizer(o);
    f.capture(1000000000L);
    o.p = new Pose(10, 0, 0);
    f.capture(2000000000L);
    assertTrue(f.addMeasurement(new Pose(6, 0, 0), 1500000000L, Matrix.diag(.01, .01, .01), 12));
    assertTrue(f.pose().x() > 10.9);
    double x = f.pose().x();
    assertFalse(f.addMeasurement(new Pose(0, 0, 0), 1400000000L, Matrix.diag(1, 1, 1), 12));
    assertEquals(x, f.pose().x(), 0);
  }
}
