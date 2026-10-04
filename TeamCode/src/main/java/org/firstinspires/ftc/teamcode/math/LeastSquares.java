package org.firstinspires.ftc.teamcode.math;

/** Small dense solver shared by calibration and vision. No hardware dependencies. */
public final class LeastSquares {
  public interface Model {
    double[] residual(double[] parameters);
  }

  public static final class Fit {
    public double[] parameters;
    public double[][] covariance;
    public double rms;
    public boolean valid;
  }

  public static Fit fit(Model model, double[] initial, int iterations) {
    double[] x = initial.clone();
    double[][] normal = null;
    for (int step = 0; step < iterations; step++) {
      double[] r = model.residual(x);
      if (r.length < x.length) break;
      double[][] j = jacobian(model, x, r);
      normal = normal(j);
      double[] b = new double[x.length];
      for (int i = 0; i < r.length; i++) for (int k = 0; k < x.length; k++) b[k] -= j[i][k] * r[i];
      double[] delta = solve(normal, b);
      if (delta == null) break;
      double cost = cost(r), scale = 1;
      boolean accepted = false;
      for (int trial = 0; trial < 12; trial++) {
        double[] next = x.clone();
        for (int k = 0; k < x.length; k++) next[k] += scale * delta[k];
        if (cost(model.residual(next)) <= cost) {
          x = next;
          accepted = true;
          break;
        }
        scale *= .5;
      }
      if (!accepted) break;
    }
    Fit f = new Fit();
    f.parameters = x;
    double[] r = model.residual(x);
    f.rms = Math.sqrt(cost(r) / Math.max(1, r.length));
    normal = normal(jacobian(model, x, r));
    f.covariance = inverse(normal);
    f.valid = f.covariance != null && Double.isFinite(f.rms);
    return f;
  }

  private static double cost(double[] r) {
    double s = 0;
    for (double v : r) s += v * v;
    return s;
  }

  private static double[][] jacobian(Model m, double[] x, double[] r) {
    double[][] j = new double[r.length][x.length];
    for (int k = 0; k < x.length; k++) {
      double[] y = x.clone();
      double e = 1e-5 * Math.max(1, Math.abs(x[k]));
      y[k] += e;
      double[] q = m.residual(y);
      for (int i = 0; i < r.length; i++) j[i][k] = (q[i] - r[i]) / e;
    }
    return j;
  }

  private static double[][] normal(double[][] j) {
    int n = j[0].length;
    double[][] a = new double[n][n];
    for (double[] row : j)
      for (int i = 0; i < n; i++) for (int k = 0; k < n; k++) a[i][k] += row[i] * row[k];
    return a;
  }

  public static double[] solve(double[][] a, double[] b) {
    int n = b.length;
    double[][] m = new double[n][n + 1];
    double max = 0;
    for (int i = 0; i < n; i++) {
      System.arraycopy(a[i], 0, m[i], 0, n);
      m[i][n] = b[i];
      max = Math.max(max, Math.abs(a[i][i]));
    }
    for (int k = 0; k < n; k++) {
      int p = k;
      for (int i = k + 1; i < n; i++) if (Math.abs(m[i][k]) > Math.abs(m[p][k])) p = i;
      if (!Double.isFinite(m[p][k]) || Math.abs(m[p][k]) < Math.max(1e-12, max * 1e-10))
        return null;
      double[] tmp = m[p];
      m[p] = m[k];
      m[k] = tmp;
      double div = m[k][k];
      for (int j = k; j <= n; j++) m[k][j] /= div;
      for (int i = 0; i < n; i++)
        if (i != k) {
          double s = m[i][k];
          for (int j = k; j <= n; j++) m[i][j] -= s * m[k][j];
        }
    }
    double[] x = new double[n];
    for (int i = 0; i < n; i++) {
      x[i] = m[i][n];
      if (!Double.isFinite(x[i])) return null;
    }
    return x;
  }

  public static double[][] inverse(double[][] a) {
    int n = a.length;
    double[][] inv = new double[n][n];
    for (int k = 0; k < n; k++) {
      double[] b = new double[n];
      b[k] = 1;
      double[] x = solve(a, b);
      if (x == null) return null;
      for (int i = 0; i < n; i++) inv[i][k] = x[i];
    }
    return inv;
  }
}
