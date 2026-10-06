package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.*;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.control.Angles;

/** Axon indexer; shooting, reverse and shutdown share one feed decision. */
public final class Indexer extends SubsystemBase {
  private final ServoEx indexer;
  private double position;

  public Indexer(HardwareMap hw) {
    indexer = new ServoEx(hw, "indexer");
    indexer.setPwm(new PwmControl.PwmRange(500, 2500));
    indexer.setInverted(false);
    retract();
  }

  public void feed(boolean feeding) {
    setPosition(
        feeding && MechanismConfig.indexerCalibrated
            ? MechanismConfig.indexerDeployed
            : MechanismConfig.indexerRetracted);
  }

  /** Manual calibration only; driver commands must use feed(). */
  public void setPosition(double p) {
    if (Double.isFinite(p)) position = Angles.clamp(p, 0, 1);
  }

  public void write() {
    indexer.set(position);
  }

  public void retract() {
    feed(false);
    write();
  }

  public double getPosition() {
    return position;
  }
}
