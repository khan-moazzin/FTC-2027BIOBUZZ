package org.firstinspires.ftc.teamcode.lib.logging;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/** Immutable scalar snapshot; no hardware objects cross the logger thread boundary. */
public final class LogFrame {
  public final long time;
  public final Map<String, Object> values;

  public LogFrame(long time, Map<String, Object> values) {
    this.time = time;
    this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
    for (Object v : this.values.values()) {
      if (v instanceof Number && !Double.isFinite(((Number) v).doubleValue()))
        throw new IllegalArgumentException("Use number() for invalid readings");
      if (!(v instanceof Number || v instanceof Boolean || v instanceof String))
        throw new IllegalArgumentException("Log values must be scalars");
    }
  }

  /**
   * Invalid readings have an explicit validity channel; zero alone is never evidence of validity.
   */
  public static void number(Map<String, Object> values, String key, double value) {
    values.put(key, Double.isFinite(value) ? value : 0.0);
    values.put(key + "/Valid", Double.isFinite(value));
  }
}
