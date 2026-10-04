package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.MechanismConfig;
import org.firstinspires.ftc.teamcode.math.LeastSquares;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

@TeleOp(name = "Tune 3 - Flywheel feedforward", group = "Calibration")
public final class FlywheelTuner extends GuidedOpMode {
  private Flywheel wheel;
  private int stage;
  private long started;
  private final List<double[]> samples = new ArrayList<>();

  protected void setup() {
    wheel = new Flywheel(hardwareMap);
  }

  protected void tick(long now) throws Exception {
    wheel.read(now);
    telemetry.addLine(
        "Empty shooter. Hold RB continuously to run automatic 20-70% power steps. Release stops.");
    telemetry.addLine(
        "A starts/restarts. Y fits each wheel independently and exports MechanismConfig.");
    if (a) {
      stage = 1;
      started = now;
      samples.clear();
    }
    double power = stage > 0 && stage <= 6 ? .1 + stage * .1 : 0;
    if (!gamepad1.right_bumper) {
      wheel.characterize(0);
      started = now;
    } else {
      wheel.characterize(power);
      if (power > 0) {
        double age = (now - started) * 1e-9;
        if (age > 1.5) samples.add(new double[] {wheel.getLeftRpm(), wheel.getRightRpm(), power});
        if (age > 2.5) {
          stage++;
          started = now;
        }
      }
    }
    telemetry.addData(
        "Stage / RPM L/R", stage + " / " + wheel.getLeftRpm() + " / " + wheel.getRightRpm());
    if (y && stage > 6) {
      for (int i = 0; i < 2; i++) {
        final int side = i;
        LeastSquares.Fit f =
            LeastSquares.fit(
                p -> {
                  double[] r = new double[samples.size()];
                  for (int j = 0; j < r.length; j++) {
                    double[] s = samples.get(j);
                    r[j] = p[0] + p[1] * s[side] - s[2];
                  }
                  return r;
                },
                new double[] {0, .0001},
                3);
        if (!f.valid || f.rms > .04 || f.parameters[1] <= 0) {
          result = "Characterization failed; check RPM sign and encoder resolution";
          return;
        }
        MechanismConfig.flywheelS[i] = Math.max(0, f.parameters[0]);
        MechanismConfig.flywheelV[i] = f.parameters[1];
        MechanismConfig.flywheelP[i] = f.parameters[1] * .25;
      }
      MechanismConfig.flywheelCalibrated = true;
      export(new MechanismConfig());
      result += "; validate loaded speed recovery with Tune 5";
    }
  }

  protected void shutdown() {
    if (wheel != null) wheel.stop();
  }
}
