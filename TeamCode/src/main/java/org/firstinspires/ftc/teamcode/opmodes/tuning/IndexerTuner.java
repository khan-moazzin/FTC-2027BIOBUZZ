package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.calibration.GuidedOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "Tune 6 - Indexer endpoints", group = "Calibration")
public final class IndexerTuner extends GuidedOpMode {
  private Indexer indexer;
  private double retracted = Double.NaN, deployed = Double.NaN;
  private long last;

  protected void setup() {
    indexer = new Indexer(hardwareMap);
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? 0 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    telemetry.addLine(
        "Empty feeder. Hold RB + left stick to move slowly. A captures RETRACTED; X captures"
            + " DEPLOYED; Y exports.");
    if (gamepad1.right_bumper) {
      indexer.setPosition(indexer.getPosition() - gamepad1.left_stick_y * .15 * dt);
      indexer.write();
    }
    if (a) retracted = indexer.getPosition();
    if (x) deployed = indexer.getPosition();
    telemetry.addData("Position", indexer.getPosition());
    telemetry.addData("Retracted / deployed", retracted + " / " + deployed);
    if (y) {
      if (!Double.isFinite(retracted)
          || !Double.isFinite(deployed)
          || Math.abs(retracted - deployed) < .02) {
        result = "Capture two distinct, physically verified endpoints";
        return;
      }
      MechanismConfig.indexerRetracted = retracted;
      MechanismConfig.indexerDeployed = deployed;
      MechanismConfig.indexerCalibrated = true;
      indexer.retract();
      export(new MechanismConfig());
    }
  }

  protected void shutdown() {
    if (indexer != null) indexer.retract();
  }
}
