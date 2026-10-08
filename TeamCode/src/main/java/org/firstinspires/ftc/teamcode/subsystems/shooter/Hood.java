package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.control.Angles;
import org.firstinspires.ftc.teamcode.lib.control.ServoSettling;

public final class Hood extends SubsystemBase {
  private final ServoEx hood;
  private double position = MechanismConfig.hoodStow;
  private final ServoSettling settling = new ServoSettling();

  public Hood(HardwareMap hw) {
    hood = new ServoEx(hw, "hood");
    hood.setPwm(new com.qualcomm.robotcore.hardware.PwmControl.PwmRange(500, 2500));
    hood.setInverted(false);
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
    hood.set(position);
    settling.written(
        System.nanoTime(),
        MechanismConfig.hoodSecondsPer60Degrees * 6,
        MechanismConfig.servoSettleMargin);
  }

  public void stow() {
    setPosition(MechanismConfig.hoodStow);
    write();
  }

  public double getPosition() {
    return position;
  }
}
