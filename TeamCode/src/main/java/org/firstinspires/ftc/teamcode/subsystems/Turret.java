package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.control.*;

public final class Turret extends SubsystemBase {
  private final ServoEx left, right;
  private final AbsoluteAnalogEncoder le, re;
  public final TurretFeedback feedback = new TurretFeedback();
  public final VisionConfig config;
  public double leftVoltage, rightVoltage;
  private double position = .5, targetAngle, lastCommandAngle;
  private boolean initialized;

  public Turret(HardwareMap hw, VisionConfig c) {
    config = c;
    left = new ServoEx(hw, "turret1");
    right = new ServoEx(hw, "turret2");
    right.setInverted(true);
    left.setPwm(new PwmControl.PwmRange(500, 2500));
    right.setPwm(new PwmControl.PwmRange(500, 2500));
    le = new AbsoluteAnalogEncoder(hw, c.leftFeedback);
    re = new AbsoluteAnalogEncoder(hw, c.rightFeedback);
    position = c.servoCenter;
  }

  public void read(long now) {
    leftVoltage = le.getVoltage();
    rightVoltage = re.getVoltage();
    feedback.update(leftVoltage, rightVoltage, now, config);
    if (feedback.healthy && !initialized) {
      double lo = (config.servoMin - config.servoCenter) * config.radiansPerServo;
      double hi = (config.servoMax - config.servoCenter) * config.radiansPerServo;
      if (TurretTarget.ambiguous(feedback.angle, lo, hi, config.feedbackTolerance)) {
        // A 1:1 absolute encoder cannot distinguish the two ends of a complete turn at startup.
        feedback.healthy = false;
        return;
      }
      lastCommandAngle = TurretTarget.measured(feedback.angle, 0, lo, hi, config.feedbackTolerance);
      targetAngle = lastCommandAngle;
      position = config.servoCenter + lastCommandAngle / config.radiansPerServo;
      initialized = true;
    }
  }

  public double aim(double angle, double rate, double dt) {
    if (!initialized
        || !feedback.healthy
        || !Double.isFinite(angle)
        || !Double.isFinite(rate)
        || !Double.isFinite(dt)) return Double.NaN;
    double lo = (config.servoMin - config.servoCenter) * config.radiansPerServo,
        hi = (config.servoMax - config.servoCenter) * config.radiansPerServo;
    double wanted = Angles.wrap(angle);
    targetAngle =
        org.firstinspires.ftc.teamcode.lib.control.TurretTarget.choose(
            wanted, lastCommandAngle, lo, hi);
    double led = Angles.clamp(targetAngle + rate * config.turretLag, lo, hi);
    lastCommandAngle +=
        Angles.clamp(
            led - lastCommandAngle,
            -config.maxTurretRate * Math.max(0, Math.min(.1, dt)),
            config.maxTurretRate * Math.max(0, Math.min(.1, dt)));
    position = config.servoCenter + lastCommandAngle / config.radiansPerServo;
    return Angles.wrap(wanted - targetAngle);
  }

  public boolean ready() {
    double lo = (config.servoMin - config.servoCenter) * config.radiansPerServo;
    double hi = (config.servoMax - config.servoCenter) * config.radiansPerServo;
    double measured =
        TurretTarget.measured(feedback.angle, lastCommandAngle, lo, hi, config.feedbackTolerance);
    return initialized
        && feedback.healthy
        && Math.abs(targetAngle - measured) < MechanismConfig.turretTolerance;
  }

  public void write() {
    if (initialized && config.turretCalibrated && feedback.healthy && Double.isFinite(position)) {
      left.set(position);
      right.set(position);
    }
  }

  public void setRawPosition(double p) {
    if (Double.isFinite(p)) {
      position = Angles.clamp(p, 0, 1);
      left.set(position);
      right.set(position);
    }
  }

  public double getAngle() {
    return Math.toDegrees(feedback.angle);
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCommandAngle() {
    return lastCommandAngle;
  }

  public double getPosition() {
    return position;
  }

  public void hold() {
    if (feedback.healthy) {
      double lo = (config.servoMin - config.servoCenter) * config.radiansPerServo;
      double hi = (config.servoMax - config.servoCenter) * config.radiansPerServo;
      lastCommandAngle =
          TurretTarget.measured(feedback.angle, lastCommandAngle, lo, hi, config.feedbackTolerance);
      targetAngle = lastCommandAngle;
      position = config.servoCenter + lastCommandAngle / config.radiansPerServo;
      write();
    }
  }
}
