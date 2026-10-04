package org.firstinspires.ftc.teamcode.vision;

import java.util.*;
import org.firstinspires.ftc.teamcode.control.Angles;

/** Measured angles only. Missing feedback deliberately breaks interpolation. */
public final class AngleHistory {
  private final int capacity;
  private final long window;

  public AngleHistory() {
    this(512, 2000000000L);
  }

  public AngleHistory(int capacity, long window) {
    this.capacity = capacity;
    this.window = window;
  }

  private final NavigableMap<Long, Double> data = new TreeMap<>();

  public void add(long t, double angle) {
    data.put(t, angle);
    while (data.size() > capacity || (!data.isEmpty() && data.firstKey() < t - window))
      data.pollFirstEntry();
  }

  public double at(long t) {
    Map.Entry<Long, Double> a = data.floorEntry(t), b = data.ceilingEntry(t);
    if (a == null || b == null || b.getKey() - a.getKey() > 100000000L) return Double.NaN;
    if (a.getKey().equals(b.getKey())) return a.getValue();
    return a.getValue()
        + Angles.wrap(b.getValue() - a.getValue())
            * (double) (t - a.getKey())
            / (b.getKey() - a.getKey());
  }

  public String csv() {
    StringBuilder s = new StringBuilder("timestamp_ns,angle_radians\n");
    for (Map.Entry<Long, Double> entry : data.entrySet())
      s.append(entry.getKey()).append(',').append(entry.getValue()).append('\n');
    return s.toString();
  }

  public void clear() {
    data.clear();
  }
}
