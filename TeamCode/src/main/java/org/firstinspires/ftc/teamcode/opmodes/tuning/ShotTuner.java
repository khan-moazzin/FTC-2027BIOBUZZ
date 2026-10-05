package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.calibration.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Tune 5 - Shooter flight map", group = "Calibration")
public final class ShotTuner extends GuidedOpMode {
  private Flywheel flywheel;
  private Hood hood;
  private Intake intake;
  private final ShotConfig c = new ShotConfig();
  private final List<double[]> samples = new ArrayList<>();
  private final double[] row = {48, 24, 3000, .6, .5};
  private int selected;
  private long last;
  private final String[] labels = {
    "horizontal distance inches",
    "target minus muzzle height inches",
    "RPM",
    "hood radians",
    "measured flight seconds"
  };

  protected void setup() {
    flywheel = new Flywheel(hardwareMap);
    hood = new Hood(hardwareMap);
    intake = new Intake(hardwareMap);
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? .02 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    flywheel.read(now);
    if (x) selected = (selected + 1) % 5;
    row[selected] -= gamepad1.left_stick_y * dt * (selected == 2 ? 500 : selected >= 3 ? .2 : 5);
    telemetry.addData("X selects; stick adjusts", labels[selected] + " = " + row[selected]);
    telemetry.addLine(
        "Hold RB prepares. RT feeds only when both wheels and hood ready. A records a CONFIRMED"
            + " successful measured shot.");
    telemetry.addLine(
        "Use video for flight time; cover multiple distances AND target heights. Y exports only a"
            + " non-collinear map.");
    boolean spin = gamepad1.right_bumper;
    double hoodPosition = MechanismConfig.hoodZero + row[3] / MechanismConfig.hoodRadiansPerUnit;
    boolean achievable =
        ShotMap.valid(row)
            && row[2] <= MechanismConfig.maxRpm
            && Double.isFinite(hoodPosition)
            && hoodPosition >= MechanismConfig.hoodMin
            && hoodPosition <= MechanismConfig.hoodMax;
    flywheel.setTargetRpm(spin && achievable ? row[2] : 0);
    if (achievable) hood.setAngle(row[3]);
    flywheel.write();
    hood.write();
    intake.set(
        spin && achievable && gamepad1.right_trigger > .5 && flywheel.atSpeed() && hood.ready(now)
            ? 1
            : 0);
    intake.write();
    telemetry.addData("RPM L/R", flywheel.getLeftRpm() + " / " + flywheel.getRightRpm());
    telemetry.addData("Samples", samples.size());
    if (!achievable) telemetry.addLine("Requested shot exceeds configured RPM/hood limits");
    if (a && achievable) samples.add(row.clone());
    if (y) {
      c.samples = samples.toArray(new double[0][]);
      c.calibrated = true;
      boolean coverage = false;
      for (double[] p : samples) if (ShotMap.lookup(c, p[0], p[1]) != null) coverage = true;
      if (!coverage) {
        result = "Need 3+ non-collinear distance/height samples";
        return;
      }
      export(c);
    }
  }

  protected void shutdown() {
    if (intake != null) intake.stop();
    if (flywheel != null) flywheel.stop();
    if (hood != null) hood.stow();
  }
}
