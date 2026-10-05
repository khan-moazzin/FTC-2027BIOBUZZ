package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.control.Angles;
import org.firstinspires.ftc.teamcode.lib.control.ServoSettling;

public final class Hood extends SubsystemBase {
  private final ServoEx servo;
  private double position = MechanismConfig.hoodStow;
  private final ServoSettling settling = new ServoSettling();

  public Hood(HardwareMap hw) {
    servo = new ServoEx(hw, "hood");
    settling.command(position);
  }

  public void setPosition(double p) {
    if (!Double.isFinite(p)) return;
    p = Angles.clamp(p, MechanismConfig.hoodMin, MechanismConfig.hoodMax);
    settling.command(p);
    position = p;
  }

  public void setAngle(double a) {
    setPosition(MechanismConfig.hoodZero + a / MechanismConfig.hoodRadiansPerUnit);
  }

  public boolean ready(long now) {
    return MechanismConfig.hoodCalibrated && settling.ready(now, MechanismConfig.hoodSettle);
  }

  public void write() {
    servo.set(position);
    settling.written(System.nanoTime());
  }

  public void stow() {
    setPosition(MechanismConfig.hoodStow);
    write();
  }

  public double getPosition() {
    return position;
  }
}
