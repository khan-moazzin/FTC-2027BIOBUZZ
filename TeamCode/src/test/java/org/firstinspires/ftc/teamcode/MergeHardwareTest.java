package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.junit.Test;

public class MergeHardwareTest {
  @Test
  public void singleWheelMustHoldSpeedAndOffOrInvalidNeverPasses() {
    assertTrue(Readiness.speed(3602, 3601.8, 116));
    assertFalse(Readiness.speed(0, 0, 116));
    assertFalse(Readiness.speed(Double.NaN, 3601.8, 116));
    assertFalse(Readiness.speed(3300, 3601.8, 116));
    Readiness dwell = new Readiness();
    assertFalse(dwell.update(true, 1000000000L, .1));
    assertTrue(dwell.update(true, 1100000000L, .1));
    assertFalse(dwell.update(false, 1110000000L, .1));
    assertFalse(dwell.update(true, 1120000000L, .1));
  }

  @Test
  public void reverseOrMissingReadinessAlwaysRetractsFeed() {
    for (int mask = 0; mask < 16; mask++) {
      boolean prepare = (mask & 1) != 0, request = (mask & 2) != 0;
      boolean ready = (mask & 4) != 0, reverse = (mask & 8) != 0;
      assertEquals(mask == 7, Readiness.feed(prepare, request, ready, reverse));
    }
  }

  @Test
  public void stationaryPolicyIsOptionalButInvalidMotionIsNeverAllowed() {
    assertFalse(Readiness.motionAllowed(true, 7, 0, 0, 6, .26));
    assertFalse(Readiness.motionAllowed(true, 0, 0, -.3, 6, .26));
    assertTrue(Readiness.motionAllowed(true, 3, 4, .2, 6, .26));
    assertTrue(Readiness.motionAllowed(false, 20, 10, 1, 6, .26));
    assertFalse(Readiness.motionAllowed(false, Double.NaN, 0, 0, 6, .26));
  }

  @Test
  public void hoodTravelTimeAndFixedDwellBothApply() {
    ServoSettling hood = new ServoSettling();
    hood.command(.15);
    hood.written(1000000000L, .66, .08);
    assertFalse(hood.ready(1500000000L, .25));
    assertTrue(hood.ready(1800000000L, .25));
    hood.command(.85);
    hood.written(2000000000L, .66, .08);
    assertFalse(hood.ready(2300000000L, .25));
    assertTrue(hood.ready(2600000000L, .25));
    hood.command(.851);
    hood.written(2700000000L, .66, .08);
    assertTrue(hood.ready(2700000000L, .25));
    hood.command(.853);
    hood.written(2800000000L, .66, .08);
    assertFalse(hood.ready(2810000000L, .25));
  }

  @Test
  public void hardwareDefaultsAndTrialDoNotEnableIncompleteCalibration() {
    assertEquals(5800, MechanismConfig.maxRpm, 0);
    assertEquals(2 * Math.PI * 30 / 173, MechanismConfig.hoodRadiansPerUnit, 1e-9);
    VisionConfig v = new VisionConfig();
    assertEquals(Math.toRadians(-170), (v.servoMin - v.servoCenter) * v.radiansPerServo, 1e-9);
    assertEquals(Math.toRadians(170), (v.servoMax - v.servoCenter) * v.radiansPerServo, 1e-9);
    ShotConfig c = new ShotConfig();
    assertEquals(3601.8, c.referenceTrial[2] / 100 * 5800, 1e-8);
    assertFalse(c.calibrated);
    assertEquals(0, c.samples.length);
    assertFalse(MechanismConfig.indexerCalibrated);
    assertFalse(MechanismConfig.kickupCalibrated);
  }
}
