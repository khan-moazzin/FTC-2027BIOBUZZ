package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.lib.calibration.*;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

@TeleOp(name = "Tune 4 - Hood angle mapping", group = "Calibration")
public final class HoodTuner extends GuidedOpMode {
  private Hood hood;
  private double measured = 30, p1, a1;
  private int stage;
  private long last;

  protected void setup() {
    hood = new Hood(hardwareMap);
  }

  protected void tick(long now) throws Exception {
    double dt = last == 0 ? .02 : Math.min(.1, (now - last) * 1e-9);
    last = now;
    telemetry.addLine(
        "Hold RB + left stick to move within configured safe hood range. Right stick sets measured"
            + " launch angle.");
    telemetry.addLine(
        "A records first position/angle. Move well apart, measure again, A fits and exports.");
    if (gamepad1.right_bumper) {
      hood.setPosition(hood.getPosition() - gamepad1.left_stick_y * .1 * dt);
      hood.write();
    }
    measured -= gamepad1.right_stick_y * 20 * dt;
    telemetry.addData("Servo / measured degrees", hood.getPosition() + " / " + measured);
    if (a) {
      if (stage == 0) {
        p1 = hood.getPosition();
        a1 = Math.toRadians(measured);
        stage++;
      } else {
        double dp = hood.getPosition() - p1;
        if (Math.abs(dp) < .1) {
          result = "Use positions at least 0.1 apart";
          return;
        }
        double slope = (Math.toRadians(measured) - a1) / dp;
        if (Math.abs(slope) < .1) {
          result = "Need distinct measured angles";
          return;
        }
        MechanismConfig.hoodRadiansPerUnit = slope;
        MechanismConfig.hoodZero = p1 - a1 / slope;
        MechanismConfig.hoodCalibrated = true;
        export(new MechanismConfig());
      }
    }
  }

  protected void shutdown() {
    if (hood != null) hood.stow();
  }
}
