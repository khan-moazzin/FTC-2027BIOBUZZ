package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.lib.math.LeastSquares;
import org.firstinspires.ftc.teamcode.lib.vision.HiveState;
import org.junit.Test;

public class SafetyTest {
  @Test
  public void hoodMustBeWrittenAndSmallMovesAccumulate() {
    ServoSettling s = new ServoSettling();
    s.command(.5);
    assertFalse(s.ready(1000000000L, .25));
    s.written(1000000000L);
    assertFalse(s.ready(1100000000L, .25));
    assertTrue(s.ready(1300000000L, .25));
    s.command(.501);
    s.written(1400000000L);
    assertTrue(s.ready(1400000000L, .25));
    s.command(.5018);
    s.written(1500000000L);
    s.command(.503);
    assertFalse(s.ready(1600000000L, .25));
    s.written(1600000000L);
    assertFalse(s.ready(1700000000L, .25));
    assertTrue(s.ready(1900000000L, .25));
    assertFalse(s.ready(1500000000L, .25));
  }

  @Test
  public void rejectedHiveCandidateCannotChangePublishedState() {
    HiveState state = new HiveState();
    assertTrue(state.update(0, .01, 1000000000L, 1));
    HiveState candidate = state.copy();
    assertTrue(candidate.update(.1, .01, 1100000000L, 1));
    // A rejected robot-pose innovation discards this candidate.
    assertEquals(0, state.angle(1100000000L), 0);
    assertEquals(0, state.rate(), 0);
    assertTrue(candidate.angle(1100000000L) > 0);
    assertFalse(state.fresh(900000000L, 1));
    assertFalse(state.update(.1, .01, 1200000000L, Double.NaN));
  }

  @Test
  public void emptyAndUnderdeterminedCalibrationAreRejected() {
    assertFalse(LeastSquares.fit(p -> new double[0], new double[] {1}, 10).valid);
    assertFalse(LeastSquares.fit(p -> new double[] {p[0]}, new double[] {1, 2}, 10).valid);
    assertFalse(LeastSquares.fit(p -> new double[] {0}, new double[] {Double.NaN}, 10).valid);
  }

  @Test
  public void unreachableTurretTargetChoosesClosestEndAcrossAngleSeam() {
    assertEquals(3, TurretTarget.choose(-3, 2, -.2, 3), 1e-9);
    assertEquals(-3, TurretTarget.choose(3, -2, -3, .2), 1e-9);
  }

  @Test
  public void invalidReadinessConfigurationCannotEnableFeed() {
    Readiness r = new Readiness();
    assertFalse(r.update(true, 1000000000L, -1));
    assertFalse(r.update(true, 1000000000L, Double.NaN));
    assertFalse(Readiness.wheels(3000, 3000, 3000, Double.POSITIVE_INFINITY));
  }
}
