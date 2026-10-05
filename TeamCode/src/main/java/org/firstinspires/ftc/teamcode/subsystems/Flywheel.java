package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.control.*;

public final class Flywheel extends SubsystemBase {
  private final MotorEx[] motors;
  private final CachedVoltage voltage;
  private final PIDFController[] pid = {
    new PIDFController(0, 0, 0, 0), new PIDFController(0, 0, 0, 0)
  };
  private final Readiness gate = new Readiness();
  private final double[] rpm = new double[2];
  private double target;
  private boolean ready;

  public Flywheel(HardwareMap hw) {
    // Resolve hardware during INIT, never in the active loop.
    VoltageSensor sensor = hw.voltageSensor.iterator().next();
    voltage = new CachedVoltage(sensor::getVoltage);
    motors = new MotorEx[] {new MotorEx(hw, "flywheel1"), new MotorEx(hw, "flywheel2")};
    for (int i = 0; i < 2; i++) {
      motors[i].motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      motors[i].setRunMode(Motor.RunMode.RawPower);
      motors[i].setInverted(i == 1);
      motors[i].setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }
  }

  public void read(long now) {
    voltage.read(now);
    for (int i = 0; i < 2; i++) rpm[i] = motors[i].getVelocity() * 60 / MechanismConfig.ticksPerRev;
    ready =
        gate.update(
            MechanismConfig.flywheelCalibrated
                && (!MechanismConfig.flywheelGainsInVolts || Double.isFinite(voltage.volts(now)))
                && Readiness.wheels(rpm[0], rpm[1], target, MechanismConfig.flywheelTolerance),
            now,
            MechanismConfig.readySeconds);
  }

  public void setTargetRpm(double r) {
    double next = Double.isFinite(r) ? Angles.clamp(r, 0, MechanismConfig.maxRpm) : 0;
    if (!Readiness.wheels(rpm[0], rpm[1], next, MechanismConfig.flywheelTolerance)) {
      ready = false;
      gate.update(false, 0, 0);
    }
    target = next;
  }

  public void write() {
    for (int i = 0; i < 2; i++) {
      pid[i].setPIDF(MechanismConfig.flywheelP[i], 0, 0, 0);
      double p =
          target > 0 && MechanismConfig.flywheelCalibrated
              ? pid[i].calculate(rpm[i], target)
                  + MechanismConfig.flywheelS[i]
                  + MechanismConfig.flywheelV[i] * target
              : 0;
      if (MechanismConfig.flywheelGainsInVolts) p = CachedVoltage.duty(p, getVoltage());
      motors[i].set(Double.isFinite(p) ? Angles.clamp(p, 0, 1) : 0);
    }
  }

  public void characterize(double p) {
    for (MotorEx m : motors) m.set(Double.isFinite(p) ? Angles.clamp(p, 0, 1) : 0);
  }

  public void stop() {
    target = 0;
    ready = false;
    gate.update(false, 0, 0);
    for (PIDFController controller : pid) controller.reset();
    for (MotorEx m : motors) m.set(0);
  }

  public boolean atSpeed() {
    return ready && (!MechanismConfig.flywheelGainsInVolts || Double.isFinite(getVoltage()));
  }

  public double getVoltage() {
    return voltage.volts(System.nanoTime());
  }

  public double getRpm() {
    return (rpm[0] + rpm[1]) / 2;
  }

  public double getLeftRpm() {
    return rpm[0];
  }

  public double getRightRpm() {
    return rpm[1];
  }

  public double getTargetRpm() {
    return target;
  }
}
