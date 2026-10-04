package org.firstinspires.ftc.teamcode.control;

import java.util.Arrays;

/** Fixed storage; sorting only when telemetry is requested. */
public final class LoopTiming {
  private final double[] samples = new double[128];
  private int count, index;
  private long start;
  public double readMs, writeMs;

  public void start(long now) {
    start = now;
  }

  public void readDone() {
    readMs = (System.nanoTime() - start) * 1e-6;
  }

  public void finish(long writeStart) {
    long now = System.nanoTime();
    writeMs = (now - writeStart) * 1e-6;
    samples[index++ % samples.length] = (now - start) * 1e-6;
    count = Math.min(samples.length, count + 1);
  }

  public String summary() {
    if (count == 0) return "No samples";
    double[] sorted = Arrays.copyOf(samples, count);
    Arrays.sort(sorted);
    return String.format(
        java.util.Locale.US,
        "p50 %.1f / p95 %.1f / max %.1f ms",
        sorted[count / 2],
        sorted[(int) ((count - 1) * .95)],
        sorted[count - 1]);
  }
}
