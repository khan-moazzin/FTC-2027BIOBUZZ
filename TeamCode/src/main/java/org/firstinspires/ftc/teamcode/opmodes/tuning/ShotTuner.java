package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.calibration.*;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;

@TeleOp(name = "Tune 5 - Shot map calibration", group = "Calibration")
public final class ShotTuner extends GuidedOpMode {
  private Flywheel flywheel;
  private Hood hood;
  private Intake intake;
  private Indexer indexer;
  private final ShotConfig c = new ShotConfig();
  private final PhysicsShotConfig physics = new PhysicsShotConfig();
  private final List<double[]> samples = new ArrayList<>();
  private final double[] row = {48, 24, 62.1, 10.61, .5};
  private int selected;
  private long last;
  private final String[] labels = {
    "horizontal distance inches",
    "target minus muzzle height inches",
    "flywheel percent (0-100)",
    "hood degrees above lowest",
    "measured flight seconds"
  };

  protected void setup() {
    flywheel = new Flywheel(hardwareMap);
    hood = new Hood(hardwareMap);
    intake = new Intake(hardwareMap);
    indexer = new Indexer(hardwareMap);
    row[0] = c.referenceTrial[0];
    row[2] = c.referenceTrial[2];
    row[3] = c.referenceTrial[1];
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? .02 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    flywheel.read(now);
    if (x) selected = (selected + 1) % 5;
    row[selected] -= gamepad1.left_stick_y * dt * (selected == 2 ? 10 : 5);
    telemetry.addData("X selects; stick adjusts", labels[selected] + " = " + row[selected]);
    telemetry.addLine(
        "Hold RB prepares. RT feeds only when flywheel, hood and calibrated indexer ready. A"
            + " records a CONFIRMED successful measured shot.");
    telemetry.addLine(
        "Stationary robot and stable raised HIVE only. A records a map point. Height and flight"
            + " time are only needed for the optional 9+ shot physics correction fit. Y exports.");
    boolean spin = gamepad1.right_bumper;
    double rpm = row[2] / 100 * MechanismConfig.maxRpm;
    double hoodAngle = Math.toRadians(row[3]);
    double hoodPosition = MechanismConfig.hoodZero + hoodAngle / MechanismConfig.hoodRadiansPerUnit;
    boolean achievable =
        Double.isFinite(row[0])
            && row[0] >= 0
            && Double.isFinite(row[2])
            && row[2] > 0
            && row[2] <= 100
            && Double.isFinite(hoodAngle)
            && hoodAngle >= 0
            && Double.isFinite(hoodPosition)
            && hoodPosition >= MechanismConfig.hoodMin
            && hoodPosition <= MechanismConfig.hoodMax;
    flywheel.setTargetRpm(spin && achievable ? rpm : 0);
    if (achievable) hood.setAngle(hoodAngle);
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
    if (a && achievable)
      samples.add(new double[] {row[0], row[1], rpm, hoodAngle, row[4]});
    if (y) {
      c.samples = samples.toArray(new double[0][]);
      c.hoodMap = map(c.samples, 3, 180 / Math.PI);
      c.flywheelMap = map(c.samples, 2, 100 / MechanismConfig.maxRpm);
      c.calibrated = ShotMap.validMap(c.hoodMap) && ShotMap.validMap(c.flywheelMap);
      if (!c.calibrated) {
        result = "Record at least one valid shot-map point before exporting";
        return;
      }
      export(c);
      String mapExport = result;
      if (samples.size() < 9) {
        result = mapExport + "; map ready, optional physics correction needs 9+ varied shots";
        return;
      }
      PhysicsCalibration.Result fit = PhysicsCalibration.fit(physics, c.samples);
      if (!fit.valid) {
        result = mapExport + "; map ready, optional physics fit unavailable: " + fit.reason;
        return;
      }
      export(physics);
      result =
          mapExport
              + "; physics correction export: "
              + result
              + "; validation max "
              + fit.validationMax
              + " m. Verify geometry, then run offline generator.";
    }
  }

  private static double[][] map(double[][] samples, int valueColumn, double scale) {
    NavigableMap<Double, Double> points = new TreeMap<>();
    for (double[] sample : samples)
      if (sample != null
          && sample.length == 5
          && Double.isFinite(sample[0])
          && sample[0] >= 0
          && Double.isFinite(sample[valueColumn]))
        points.put(sample[0], sample[valueColumn] * scale);
    double[][] map = new double[points.size()][2];
    int index = 0;
    for (Map.Entry<Double, Double> point : points.entrySet())
      map[index++] = new double[] {point.getKey(), point.getValue()};
    return map;
  }

  protected void shutdown() {
    if (intake != null) intake.stop();
    if (indexer != null) indexer.retract();
    if (flywheel != null) flywheel.stop();
    if (hood != null) hood.stow();
  }
}
