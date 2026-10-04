package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.localization.*;
import com.pedropathing.math.*;
import java.util.*;
import org.firstinspires.ftc.teamcode.control.Angles;

/**
 * Bounded delayed EKF. capture() reads hardware once; update() exposes the prepared state to Pedro.
 */
public final class BufferedFusionLocalizer implements Localizer {
  private static final class Sample {
    Pose raw, pose;
    Velocity velocity;
    Matrix covariance;

    Sample(Pose r, Pose p, Velocity v, Matrix c) {
      raw = r;
      pose = p;
      velocity = v;
      covariance = c;
    }
  }

  private final Localizer raw;
  private final NavigableMap<Long, Sample> history = new TreeMap<>();
  private MotionState state = MotionState.zero();
  private Pose lastRaw = Pose.zero();
  private Matrix covariance = initial();
  private long lastTime = -1, lastMeasurement = -1;
  public String status = "Waiting for odometry";

  private static Matrix initial() {
    return Matrix.diag(4, 4, Math.pow(Math.toRadians(10), 2));
  }

  public BufferedFusionLocalizer(Localizer raw) {
    this.raw = raw;
    covariance = Matrix.diag(72 * 72, 72 * 72, Math.PI * Math.PI);
    lastRaw = raw.pose();
    state = MotionState.ofVelocity(lastRaw, Velocity.zero());
  }

  public void capture(long now) {
    raw.update();
    Pose current = raw.pose();
    if (!finite(current)) {
      status = "Nonfinite odometry";
      return;
    }
    Pose next = state.pose().compose(lastRaw.invert().compose(current));
    double dt = lastTime < 0 ? 0 : (now - lastTime) * 1e-9;
    covariance = propagate(covariance, state.pose(), next, dt);
    lastRaw = current;
    lastTime = now;
    state =
        MotionState.ofVelocity(
            next, rotated(raw.velocity(), Angles.wrap(next.heading() - current.heading())));
    history.put(now, new Sample(current, next, raw.velocity(), covariance));
    prune(now);
  }

  private static Matrix propagate(Matrix p, Pose a, Pose b, double dt) {
    double dx = b.x() - a.x(), dy = b.y() - a.y();
    Matrix f = new Matrix(new double[][] {{1, 0, -dy}, {0, 1, dx}, {0, 0, 1}});
    double distance = Math.hypot(dx, dy);
    return f.times(p)
        .times(f.transpose())
        .plus(
            Matrix.diag(
                .002 * dt + .02 * distance,
                .002 * dt + .02 * distance,
                .00001 * dt + .002 * Math.abs(Angles.wrap(b.heading() - a.heading()))));
  }

  private static Velocity rotated(Velocity v, double h) {
    double c = Math.cos(h), s = Math.sin(h);
    return new Velocity(c * v.vx - s * v.vy, s * v.vx + c * v.vy, v.omega);
  }

  private void prune(long now) {
    while (history.size() > 512 || (!history.isEmpty() && history.firstKey() < now - 2000000000L))
      history.pollFirstEntry();
  }

  public Pose sample(long t) {
    Sample s = interpolate(t);
    return s == null ? null : s.pose;
  }

  private Sample interpolate(long t) {
    Map.Entry<Long, Sample> a = history.floorEntry(t), b = history.ceilingEntry(t);
    if (a == null || b == null) return null;
    if (a.getKey().equals(b.getKey())) return a.getValue();
    double f = (double) (t - a.getKey()) / (b.getKey() - a.getKey());
    Sample x = a.getValue(), y = b.getValue();
    Pose rp = Pose.interpolate(x.raw, y.raw, f), p = x.pose.compose(x.raw.invert().compose(rp));
    return new Sample(
        rp, p, x.velocity, propagate(x.covariance, x.pose, p, (t - a.getKey()) * 1e-9));
  }

  public boolean addMeasurement(Pose z, long t, Matrix r, double gate) {
    // A late older frame must not erase a newer accepted correction.
    if (t < lastMeasurement || !finite(z)) {
      status = "Out of order/nonfinite frame";
      return false;
    }
    Sample m = interpolate(t);
    if (m == null) {
      status = "Frame outside history";
      return false;
    }
    Matrix inv = m.covariance.plus(r).invert();
    if (inv == null) return false;
    com.pedropathing.math.Vector e =
        new com.pedropathing.math.Vector(
            z.x() - m.pose.x(), z.y() - m.pose.y(), Angles.wrap(z.heading() - m.pose.heading()));
    com.pedropathing.math.Vector w = inv.times(e);
    double score = 0;
    for (int i = 0; i < 3; i++) score += e.get(i) * w.get(i);
    if (!Double.isFinite(score) || score > gate) {
      status = "Innovation rejected";
      return false;
    }
    Matrix k = m.covariance.times(inv), ik = Matrix.identity(3).minus(k);
    com.pedropathing.math.Vector d = k.times(e);
    m =
        new Sample(
            m.raw,
            new Pose(m.pose.x() + d.get(0), m.pose.y() + d.get(1), m.pose.heading() + d.get(2)),
            m.velocity,
            ik.times(m.covariance).times(ik.transpose()).plus(k.times(r).times(k.transpose())));
    history.put(t, m);
    Sample previous = m;
    long previousTime = t;
    for (Map.Entry<Long, Sample> entry : history.tailMap(t, false).entrySet()) {
      Sample s = entry.getValue();
      Pose p = previous.pose.compose(previous.raw.invert().compose(s.raw));
      s.covariance =
          propagate(previous.covariance, previous.pose, p, (entry.getKey() - previousTime) * 1e-9);
      s.pose = p;
      previous = s;
      previousTime = entry.getKey();
    }
    Sample latest = history.lastEntry().getValue();
    covariance = latest.covariance;
    state =
        MotionState.ofVelocity(
            latest.pose,
            rotated(latest.velocity, Angles.wrap(latest.pose.heading() - latest.raw.heading())));
    lastMeasurement = t;
    prune(lastTime);
    status = "Accepted";
    return true;
  }

  public double positionSigma() {
    return Math.sqrt(Math.max(covariance.get(0, 0), covariance.get(1, 1)));
  }

  public int historySize() {
    return history.size();
  }

  public static boolean finite(Pose p) {
    return Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading());
  }

  @Override
  public void update() {}

  @Override
  public MotionState state() {
    return state;
  }

  @Override
  public void reset() {
    raw.reset();
    setPose(raw.pose());
  }

  @Override
  public void setPose(Pose p) {
    raw.setPose(p);
    lastRaw = p;
    state = MotionState.ofVelocity(p, Velocity.zero());
    covariance = initial();
    history.clear();
    lastTime = lastMeasurement = -1;
  }
}
