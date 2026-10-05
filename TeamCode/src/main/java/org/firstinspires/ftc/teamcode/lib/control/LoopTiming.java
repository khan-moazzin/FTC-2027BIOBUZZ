package org.firstinspires.ftc.teamcode.lib.control;

import java.io.IOException;
import java.io.Writer;
import java.util.Arrays;
import java.util.Locale;

/** Bounded monotonic stage measurements; no file I/O or sorting in the control loop. */
public final class LoopTiming {
  private static final int CAPACITY = 512;
  // Start time, read, commands, write, work, start-to-start period (nanoseconds).
  private final long[][] samples = new long[CAPACITY][6];
  private int count, index;
  private long start, readEnd;
  private boolean completed;
  public double readMs, writeMs;

  public void start(long now) {
    if (completed && now >= start) samples[(index + CAPACITY - 1) % CAPACITY][5] = now - start;
    start = now;
    completed = false;
  }

  public void readDone() {
    readDone(System.nanoTime());
  }

  public void readDone(long now) {
    readEnd = now;
    readMs = (now - start) * 1e-6;
  }

  public void finish(long writeStart) {
    finish(writeStart, System.nanoTime());
  }

  public void finish(long writeStart, long now) {
    if (completed || readEnd < start || writeStart < readEnd || now < writeStart) return;
    long[] row = samples[index];
    row[0] = start;
    row[1] = readEnd - start;
    row[2] = writeStart - readEnd;
    row[3] = now - writeStart;
    row[4] = now - start;
    row[5] = -1; // Last loop has no following start; export it as blank.
    writeMs = row[3] * 1e-6;
    index = (index + 1) % CAPACITY;
    count = Math.min(CAPACITY, count + 1);
    completed = true;
  }

  public String summary() {
    return "work " + summary(4) + "; period " + summary(5);
  }

  private String summary(int column) {
    long[] sorted = new long[count];
    int n = 0;
    for (int i = 0; i < count; i++) if (samples[i][column] >= 0) sorted[n++] = samples[i][column];
    if (n == 0) return "no samples";
    Arrays.sort(sorted, 0, n);
    return String.format(
        Locale.US,
        "p50 %.1f / p95 %.1f / max %.1f ms",
        sorted[(int) Math.ceil(n * .5) - 1] * 1e-6,
        sorted[(int) Math.ceil(n * .95) - 1] * 1e-6,
        sorted[n - 1] * 1e-6);
  }

  /** Call after actuator shutdown on the OpMode thread, never from a background singleton. */
  public void writeCsv(Writer writer) throws IOException {
    writer.write("start_ns,read_ms,commands_ms,write_ms,work_ms,period_ms\n");
    for (int i = 0; i < count; i++) {
      long[] row = samples[(index - count + CAPACITY + i) % CAPACITY];
      writer.write(Long.toString(row[0]));
      for (int j = 1; j < row.length; j++) {
        writer.write(',');
        if (row[j] >= 0) writer.write(Double.toString(row[j] * 1e-6));
      }
      writer.write('\n');
    }
  }
}
