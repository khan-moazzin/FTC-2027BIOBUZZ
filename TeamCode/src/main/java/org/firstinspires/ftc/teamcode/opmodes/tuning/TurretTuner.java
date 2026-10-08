package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.calibration.*;
import org.firstinspires.ftc.teamcode.lib.control.Angles;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;

@TeleOp(name = "Tune 1 - Turret feedback and lag", group = "Calibration")
public final class TurretTuner extends GuidedOpMode {
  private Turret turret;
  private final VisionConfig c = new VisionConfig();
  private int stage;
  private double pos = .5, leftCenter, rightCenter, lastTime, maxRate;
  private double low, high;
  private final List<double[]> points = new ArrayList<>();

  protected void setup() {
    turret = new Turret(hardwareMap, c);
    pos = turret.getPosition();
  }

  protected void tick(long now) throws Exception {
    turret.read(now);
    double t = now * 1e-9, dt = lastTime == 0 ? .02 : Math.min(.1, t - lastTime);
    lastTime = t;
    telemetry.addData("Stage", stage);
    telemetry.addData(
        "PWM / volts", "%.4f / %.3f / %.3f", pos, turret.leftVoltage, turret.rightVoltage);
    telemetry.addLine(
        "Hold RB to move slowly with left stick X. Release RB to hold. Never force a stop.");
    if (gamepad1.right_bumper) {
      pos =
          Angles.clamp(
              pos + gamepad1.left_stick_x * .1 * dt,
              stage >= 4
                  ? low
                  : Math.max(
                      0, c.servoCenter + MechanismConfig.turretMinRadians / c.radiansPerServo),
              stage >= 4
                  ? high
                  : Math.min(
                      1, c.servoCenter + MechanismConfig.turretMaxRadians / c.radiansPerServo));
      turret.setRawPosition(pos);
    }
    if (stage == 0) {
      telemetry.addLine(
          "Point turret physically forward. A captures the 1:1 encoder zero and PWM center.");
      if (a) {
        if (!Double.isFinite(turret.leftVoltage)
            || !Double.isFinite(turret.rightVoltage)
            || turret.leftVoltage <= .001
            || turret.rightVoltage <= .001) {
          result = "Check both analog feedback connections";
          return;
        }
        c.servoCenter = pos;
        c.calibrated = false; // Changing turret zero invalidates the previous camera extrinsics.
        leftCenter = turret.leftVoltage;
        rightCenter = turret.rightVoltage;
        c.leftZero = leftCenter / c.analogRange * 2 * Math.PI;
        c.rightZero = rightCenter / c.analogRange * 2 * Math.PI;
        stage++;
      }
    } else if (stage == 1) {
      telemetry.addLine(
          "Move a small amount physically CCW (20-60 degrees). A detects both encoder directions.");
      if (a) {
        double l = Angles.wrap((turret.leftVoltage - leftCenter) / c.analogRange * 2 * Math.PI),
            r = Angles.wrap((turret.rightVoltage - rightCenter) / c.analogRange * 2 * Math.PI);
        if (Math.abs(l) < .15 || Math.abs(r) < .15) {
          result = "Move farther before capture";
          return;
        }
        c.leftSign = Math.signum(l);
        c.rightSign = Math.signum(r);
        c.radiansPerServo = Math.abs(l / (pos - c.servoCenter));
        if (!Double.isFinite(c.radiansPerServo) || pos <= c.servoCenter) {
          result = "PWM must increase for CCW. Correct servo inversion first.";
          return;
        }
        c.turretCalibrated = true;
        stage++;
      }
    } else if (stage == 2) {
      telemetry.addLine("Move to minimum safe travel. A records the software limit.");
      if (a) {
        low = pos;
        stage++;
      }
    } else if (stage == 3) {
      telemetry.addLine("Move to maximum safe travel. A records the software limit.");
      if (a && pos > low + .05) {
        high = pos;
        c.servoMin = low;
        c.servoMax = high;
        stage++;
      }
    } else if (stage == 4) {
      telemetry.addLine(
          "Sweep back and forth across safe travel twice. Samples fit angle/PWM and lag; A fits"
              + " after 80+ healthy samples.");
      if (gamepad1.right_bumper && turret.feedback.healthy && pos >= low && pos <= high) {
        double angle =
            org.firstinspires.ftc.teamcode.lib.control.TurretTarget.measured(
                turret.feedback.angle,
                (pos - c.servoCenter) * c.radiansPerServo,
                (low - c.servoCenter) * c.radiansPerServo,
                (high - c.servoCenter) * c.radiansPerServo,
                c.feedbackTolerance);
        double rate = turret.feedback.velocity;
        maxRate = Math.max(maxRate, Math.abs(rate));
        points.add(new double[] {pos - c.servoCenter, angle, rate});
        if (points.size() > 2000) points.remove(0);
      }
      telemetry.addData("Samples", points.size());
      if (a && points.size() >= 80) {
        org.firstinspires.ftc.teamcode.lib.math.LeastSquares.Fit fit =
            org.firstinspires.ftc.teamcode.lib.math.LeastSquares.fit(
                v -> {
                  double[] residual = new double[points.size()];
                  for (int i = 0; i < residual.length; i++) {
                    double[] p = points.get(i);
                    residual[i] = v[0] * p[0] - v[1] * p[2] - p[1];
                  }
                  return residual;
                },
                new double[] {c.radiansPerServo, 0},
                5);
        if (!fit.valid
            || fit.rms > .08
            || fit.parameters[0] <= 0
            || fit.parameters[1] < 0
            || fit.parameters[1] > .5) {
          result = "Sweep fit rejected; repeat slower in both directions";
          return;
        }
        c.radiansPerServo = fit.parameters[0];
        c.turretLag = fit.parameters[1];
        c.maxTurretRate = Math.max(.1, maxRate * .7);
        stage++;
        export(c);
      }
    } else
      telemetry.addLine("Complete. Copy VisionConfig.java, rebuild, then run vision calibration.");
  }

  protected void shutdown() {
    if (turret != null) turret.hold();
  }
}
