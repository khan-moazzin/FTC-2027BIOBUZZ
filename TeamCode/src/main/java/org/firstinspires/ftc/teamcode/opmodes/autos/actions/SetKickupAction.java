package org.firstinspires.ftc.teamcode.opmodes.autos.actions;

import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.Kickup;

/** Sets the autonomous kickup position and leaves it there until the next kickup action. */
public final class SetKickupAction extends InstantCommand {
  public SetKickupAction(Kickup kickup, boolean raised) {
    super(
        () -> {
          if (raised) kickup.up();
          else kickup.down();
        },
        kickup);
  }
}
