package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.junit.Test;

public class ControlsTest {
  @Test
  public void intakeRestoresAfterOverride() {
    assertEquals(-.9, Intake.requested(true, true), 0);
    assertEquals(1, Intake.requested(true, false), 0);
    assertEquals(0, Intake.requested(false, false), 0);
  }

  @Test
  public void bothWheelsMustMatch() {
    assertFalse(Readiness.wheels(2500, 3500, 3000, 75));
    assertFalse(Readiness.wheels(Double.NaN, 3000, 3000, 75));
    assertTrue(Readiness.wheels(2980, 3020, 3000, 75));
  }

  @Test
  public void dwellResetsImmediately() {
    Readiness r = new Readiness();
    assertFalse(r.update(true, 1000000000L, .15));
    assertTrue(r.update(true, 1200000000L, .15));
    assertFalse(r.update(false, 1200000001L, .15));
  }

  @Test
  public void independentAnalogCalibrationAndWrap() {
    VisionConfig c = new VisionConfig();
    c.turretCalibrated = true;
    c.leftSign = c.rightSign = 1;
    TurretFeedback f = new TurretFeedback();
    f.update(3.29, 3.29, 1000000000L, c);
    f.update(.01, .01, 1020000000L, c);
    assertTrue(f.healthy);
    assertTrue(Math.abs(f.velocity) < 3);
    f.update(.01, 1, 1040000000L, c);
    assertFalse(f.healthy);
  }
}
