package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.control.*;

/** One direct-drive goBILDA motor, as configured on the physical robot. */
public final class Flywheel extends SubsystemBase {
  private final MotorEx motor;
  private final CachedVoltage voltage;
  private final PIDFController pid = new PIDFController(0, 0, 0, 0);
  private final Readiness gate = new Readiness();
  private double rpm, duty, target;
  private boolean ready;

  public Flywheel(HardwareMap hw) {
    VoltageSensor sensor = hw.voltageSensor.iterator().next();
    voltage = new CachedVoltage(sensor::getVoltage);
    motor = new MotorEx(hw, "flywheel");
    motor.motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motor.setRunMode(Motor.RunMode.RawPower);
    motor.setInverted(false);
    motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
  }

  public void read(long now) {
    voltage.read(now);
    rpm = motor.getVelocity() * 60 / MechanismConfig.ticksPerRev;
    ready =
        gate.update(
            MechanismConfig.flywheelCalibrated
                && (!MechanismConfig.flywheelGainsInVolts || Double.isFinite(voltage.volts(now)))
                && Readiness.speed(rpm, target, MechanismConfig.flywheelTolerance),
            now,
            MechanismConfig.readySeconds);
  }

  public void setTargetRpm(double r) {
    double next = Double.isFinite(r) ? Angles.clamp(r, 0, MechanismConfig.maxRpm) : 0;
    if (!Readiness.speed(rpm, next, MechanismConfig.flywheelTolerance)) {
      ready = false;
      gate.update(false, 0, 0);
    }
    target = next;
  }

  public void write() {
    pid.setPIDF(MechanismConfig.flywheelP, 0, 0, 0);
    double p =
        target > 0 && MechanismConfig.flywheelCalibrated
            ? pid.calculate(rpm, target)
                + MechanismConfig.flywheelS
                + MechanismConfig.flywheelV * target
            : 0;
    if (MechanismConfig.flywheelGainsInVolts) p = CachedVoltage.duty(p, getVoltage());
    characterize(p);
  }

  public void characterize(double p) {
    duty = Double.isFinite(p) ? Angles.clamp(p, 0, 1) : 0;
    motor.set(duty);
  }

  public void stop() {
    target = 0;
    ready = false;
    gate.update(false, 0, 0);
    pid.reset();
    characterize(0);
  }

  public boolean atSpeed() {
    return ready && (!MechanismConfig.flywheelGainsInVolts || Double.isFinite(getVoltage()));
  }

  public double duty() {
    return duty;
  }

  public double getVoltage() {
    return voltage.volts(System.nanoTime());
  }

  public double getRpm() {
    return rpm;
  }

  public double getTargetRpm() {
    return target;
  }
}
