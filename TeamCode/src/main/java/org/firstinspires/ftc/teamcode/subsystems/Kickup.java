package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.control.Angles;

public final class Kickup extends SubsystemBase {
  private final ServoEx left, right;
  private boolean up;
  private double position = MechanismConfig.kickupDown;

  public Kickup(HardwareMap hw) {
    left = new ServoEx(hw, "kickup1");
    right = new ServoEx(hw, "kickup2");
    right.setInverted(true);
    left.setPwm(new PwmControl.PwmRange(500, 2500));
    right.setPwm(new PwmControl.PwmRange(500, 2500));
    down();
  }

  public void up() {
    up = true;
    position = MechanismConfig.kickupUp;
  }

  public void down() {
    up = false;
    position = MechanismConfig.kickupDown;
  }

  public void write() {
    if (!MechanismConfig.kickupCalibrated || !Double.isFinite(position)) return;
    double command = Angles.clamp(position, 0, 1);
    left.set(command);
    right.set(command);
  }

  public void stop() {
    up();
    write();
  }

  public boolean isUp() {
    return up;
  }

  public double getPosition() {
    return position;
  }
}
