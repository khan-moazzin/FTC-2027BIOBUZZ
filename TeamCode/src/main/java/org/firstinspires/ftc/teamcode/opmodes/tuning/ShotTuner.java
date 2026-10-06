package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.calibration.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name = "Tune 5 - Physics shot calibration", group = "Calibration")
public final class ShotTuner extends GuidedOpMode {
  private Flywheel flywheel;
  private Hood hood;
  private Intake intake;
  private Indexer indexer;
  private final ShotConfig c = new ShotConfig();
  private final PhysicsShotConfig physics = new PhysicsShotConfig();
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
    indexer = new Indexer(hardwareMap);
    row[0] = c.referenceTrial[0];
    row[2] = c.referenceTrial[2] / 100 * 5800;
    // Convert the old mechanical-zero hood angle into the calibrated absolute launch frame.
    row[3] =
        (MechanismConfig.hoodMin - MechanismConfig.hoodZero) * MechanismConfig.hoodRadiansPerUnit
            + Math.toRadians(c.referenceTrial[1]);
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? .02 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    flywheel.read(now);
    if (x) selected = (selected + 1) % 5;
    row[selected] -= gamepad1.left_stick_y * dt * (selected == 2 ? 500 : selected >= 3 ? .2 : 5);
    telemetry.addData("X selects; stick adjusts", labels[selected] + " = " + row[selected]);
    telemetry.addLine(
        "Hold RB prepares. RT feeds only when flywheel, hood and calibrated indexer ready. A"
            + " records a CONFIRMED successful measured shot.");
    telemetry.addLine(
        "Stationary robot and stable raised HIVE only. Measure flight time with video. Y exports a"
            + " physics fit after 9+ varied shots, including held-out validation.");
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
    boolean feed =
        spin
            && achievable
            && gamepad1.right_trigger > .5
            && flywheel.atSpeed()
            && hood.ready(now)
            && MechanismConfig.indexerCalibrated;
    indexer.feed(feed);
    indexer.write();
    intake.set(feed ? 1 : 0);
    intake.write();
    telemetry.addData("RPM", flywheel.getRpm());
    if (!MechanismConfig.indexerCalibrated)
      telemetry.addLine("Run Tune 6 to calibrate indexer before feeding");
    telemetry.addData("Samples", samples.size());
    if (!achievable) telemetry.addLine("Requested shot exceeds configured RPM/hood limits");
    if (a && achievable) samples.add(row.clone());
    if (y) {
      c.samples = samples.toArray(new double[0][]);
      c.calibrated = false; // Archive measured samples; runtime no longer uses the empirical map.
      export(c);
      PhysicsCalibration.Result fit = PhysicsCalibration.fit(physics, c.samples);
      if (!fit.valid) {
        result = fit.reason + "; measured samples saved in ShotConfig";
        return;
      }
      export(physics);
      result +=
          "; validation max "
              + fit.validationMax
              + " m. Verify geometry, then run offline generator.";
    }
  }

  protected void shutdown() {
    if (intake != null) intake.stop();
    if (indexer != null) indexer.retract();
    if (flywheel != null) flywheel.stop();
    if (hood != null) hood.stow();
  }
}
