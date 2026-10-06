import java.nio.file.*;
import java.util.*;
import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.lib.calibration.ConfigSource;
import org.firstinspires.ftc.teamcode.lib.control.*;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.*;

/** Offline search inspired by 4414's published 2026 method. Original BIOBUZZ implementation. */
public final class GenerateShotModel {
  private static double clear(PhysicsShotConfig c) {
    // Conservative horizontal projection during training; runtime rechecks the actual tilted plane.
    return (c.openingRadius - c.projectileRadius - c.clearanceMargin) * Math.cos(Field.MAX_ANGLE);
  }

  private static double margin(
      double speed, double angle, double distance, double radial, PhysicsShotConfig c) {
    double worst = 0, height = c.targetHeight - c.muzzleHeight;
    for (int sv = -1; sv <= 1; sv++)
      for (int av = -1; av <= 1; av++) {
        double v = speed * (1 + sv * c.speedSigmaFraction * c.errorMultiplier),
            a = angle + av * c.angleSigma * c.errorMultiplier;
        Ballistics.Crossing hit =
            Ballistics.cross(
                new Vec3(0, 0, 0),
                new Vec3(v * Math.cos(a) + radial, 0, v * Math.sin(a)),
                new Vec3(distance, 0, height),
                new Vec3(0, 0, 1),
                c.dragPerMeter,
                c.maxFlightSeconds);
        if (hit == null) return Double.NEGATIVE_INFINITY;
        double lateral = distance * Math.tan(c.angleSigma * c.errorMultiplier);
        worst = Math.max(worst, Math.hypot(hit.miss, lateral));
      }
    return clear(c) - worst;
  }

  private static double[] search(double distance, double radial, PhysicsShotConfig c) {
    double height = c.targetHeight - c.muzzleHeight, best = -Double.MAX_VALUE;
    double[] result = null;
    for (int ai = 1; ai < 90; ai++) {
      double angle = c.minLaunchAngle + (c.maxLaunchAngle - c.minLaunchAngle) * ai / 90;
      double low = .01, high = Math.min(c.maxExitSpeed, c.speedPerRpm * MechanismConfig.maxRpm);
      double[] upper =
          Ballistics.atDistance(high, angle, radial, distance, c.dragPerMeter, c.maxFlightSeconds);
      if (upper == null || upper[0] < height) continue;
      for (int i = 0; i < 30; i++) {
        double speed = (low + high) / 2;
        double[] shot =
            Ballistics.atDistance(
                speed, angle, radial, distance, c.dragPerMeter, c.maxFlightSeconds);
        if (shot == null || shot[0] < height) low = speed;
        else high = speed;
      }
      double speed = (low + high) / 2;
      double[] shot =
          Ballistics.atDistance(speed, angle, radial, distance, c.dragPerMeter, c.maxFlightSeconds);
      if (shot == null || shot[2] >= 0)
        continue; // Entry from above, never an ascending rim crossing.
      double score = margin(speed, angle, distance, radial, c);
      if (score > 0 && score > best) {
        best = score;
        result = new double[] {speed, angle, shot[1]};
      }
    }
    return result;
  }

  private static double[] fit(List<double[]> inputs, List<double[]> outputs, int column) {
    LeastSquares.Fit fit =
        LeastSquares.fit(
            p -> {
              double[] residual = new double[inputs.size()];
              for (int i = 0; i < residual.length; i++) {
                residual[i] = -outputs.get(i)[column];
                for (int j = 0; j < 6; j++) residual[i] += p[j] * inputs.get(i)[j];
              }
              return residual;
            },
            new double[6],
            3);
    if (!fit.valid) throw new IllegalStateException("Polynomial fit singular");
    return fit.parameters;
  }

  public static void main(String[] args) throws Exception {
    boolean demo = Arrays.asList(args).contains("--demo");
    Path output =
        Paths.get(
            args.length == 0 || args[0].startsWith("--") ? "build/generated-physics" : args[0]);
    PhysicsShotConfig c = new PhysicsShotConfig();
    if (demo) {
      c.calibrated = c.geometryVerified = true;
      c.speedPerRpm = .002;
      c.muzzleHeight = .4;
      c.targetHeight = 1.2;
      c.openingRadius = .35;
      c.projectileRadius = .04;
      c.minDistance = 1.6;
      c.maxDistance = 2.4;
      c.maxRadialSpeed = .2;
      c.speedSigmaFraction = .005;
      c.angleSigma = Math.toRadians(.15);
    } else {
      VisionConfig vision = new VisionConfig();
      ShotConfig shots = new ShotConfig();
      c.muzzleHeight = (vision.robotToTurret[2] + shots.turretToMuzzle[2]) * .0254;
      c.targetHeight =
          Field.cell(Field.Hive.RED, Field.Cell.SCORING, Field.MAX_ANGLE).z * .0254
              + new Vec3(c.apertureOffset).rotateX(Field.MAX_ANGLE).z;
      if (!vision.calibrated || !vision.turretCalibrated || !MechanismConfig.hoodCalibrated)
        throw new IllegalStateException("Copy verified vision/turret/hood calibration first");
      c.minLaunchAngle =
          Math.max(
              c.minLaunchAngle,
              (MechanismConfig.hoodMin - MechanismConfig.hoodZero)
                      * MechanismConfig.hoodRadiansPerUnit
                  + c.launchAngleOffset);
      c.maxLaunchAngle =
          Math.min(
              c.maxLaunchAngle,
              (MechanismConfig.hoodMax - MechanismConfig.hoodZero)
                      * MechanismConfig.hoodRadiansPerUnit
                  + c.launchAngleOffset);
    }
    if (!c.calibrated || !c.geometryVerified || !ShotPolynomial.physicalInputsValid(c))
      throw new IllegalStateException(
          "Fit physics calibration and verify aperture/projectile geometry before generation");
    List<double[]> inputs = new ArrayList<>(), outputs = new ArrayList<>();
    for (int d = 0; d < 11; d++)
      for (int v = 0; v < 9; v++) {
        double distance = c.minDistance + (c.maxDistance - c.minDistance) * d / 10;
        double radial = -c.maxRadialSpeed + 2 * c.maxRadialSpeed * v / 8;
        double[] shot = search(distance, radial, c);
        if (shot == null)
          throw new IllegalStateException(
              "No robust shot at distance="
                  + distance
                  + ", radial="
                  + radial
                  + "; narrow domain or inspect calibrated uncertainty/limits");
        inputs.add(ShotPolynomial.basis(distance, radial, c));
        outputs.add(shot);
      }
    c.speedCoefficients = fit(inputs, outputs, 0);
    c.angleCoefficients = fit(inputs, outputs, 1);
    c.flightCoefficients = fit(inputs, outputs, 2);
    StringBuilder csv =
        new StringBuilder("distance_m,radial_m_s,speed_m_s,angle_rad,flight_s,miss_m,margin_m\n");
    double maxMiss = 0, minMargin = Double.POSITIVE_INFINITY;
    // Midpoints were not used for fitting. Include boundaries as well.
    for (int d = 0; d <= 20; d++)
      for (int v = 0; v <= 16; v++) {
        double distance = c.minDistance + (c.maxDistance - c.minDistance) * d / 20;
        double radial = -c.maxRadialSpeed + 2 * c.maxRadialSpeed * v / 16;
        double[] b = ShotPolynomial.basis(distance, radial, c);
        double speed = ShotPolynomial.evaluate(c.speedCoefficients, b),
            angle = ShotPolynomial.evaluate(c.angleCoefficients, b);
        double flight = ShotPolynomial.evaluate(c.flightCoefficients, b);
        Ballistics.Crossing hit =
            Ballistics.cross(
                new Vec3(0, 0, 0),
                new Vec3(speed * Math.cos(angle) + radial, 0, speed * Math.sin(angle)),
                new Vec3(distance, 0, c.targetHeight - c.muzzleHeight),
                new Vec3(0, 0, 1),
                c.dragPerMeter,
                c.maxFlightSeconds);
        double margin = margin(speed, angle, distance, radial, c);
        if (hit == null
            || hit.miss > c.modelMaxMiss
            || margin <= 0
            || speed <= 0
            || speed > c.maxExitSpeed
            || speed / c.speedPerRpm > MechanismConfig.maxRpm
            || angle < c.minLaunchAngle
            || angle > c.maxLaunchAngle
            || flight <= 0
            || flight > c.maxFlightSeconds)
          throw new IllegalStateException(
              "Polynomial validation failed at distance="
                  + distance
                  + ", radial="
                  + radial
                  + "; speed="
                  + speed
                  + ", angle="
                  + angle
                  + ", miss="
                  + (hit == null ? "none" : hit.miss)
                  + ", margin="
                  + margin
                  + "; narrow domain/refit, do not deploy");
        maxMiss = Math.max(maxMiss, hit.miss);
        minMargin = Math.min(minMargin, margin);
        csv.append(
            distance + "," + radial + "," + speed + "," + angle + "," + flight + "," + hit.miss
                + "," + margin + "\n");
      }
    c.modelGenerated = !demo;
    c.modelSignature = ShotPolynomial.signature(c);
    if (demo) c.calibrated = c.geometryVerified = false; // Synthetic output cannot arm a robot.
    Files.createDirectories(output);
    Files.write(
        output.resolve("PhysicsShotConfig.java"),
        ConfigSource.source(c).getBytes(java.nio.charset.StandardCharsets.UTF_8));
    Files.write(
        output.resolve("validation.csv"),
        csv.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8));
    String report =
        (demo
                ? "SYNTHETIC DEMO - NOT A ROBOT CALIBRATION\n"
                : "Generated from configured calibration\n")
            + "99 training points; 357 validation points. Max nominal miss="
            + maxMiss
            + " m; minimum projected margin="
            + minMargin
            + " m\n";
    Files.write(
        output.resolve("report.txt"), report.getBytes(java.nio.charset.StandardCharsets.UTF_8));
    System.out.println(report + "Output: " + output.toAbsolutePath());
  }
}
