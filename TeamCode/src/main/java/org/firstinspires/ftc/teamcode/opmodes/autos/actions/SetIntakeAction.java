package org.firstinspires.ftc.teamcode.opmodes.autos.actions;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/** Sets the autonomous intake state and leaves it there until the next mechanism action. */
public final class SetIntakeAction extends InstantCommand {
  public SetIntakeAction(Intake intake, boolean enabled) {
    super(() -> intake.set(enabled ? 1 : 0), intake);
  }
}
