package org.firstinspires.ftc.teamcode.lib.calibration;

import java.lang.reflect.*;
import java.util.*;

/** Java source serializer usable from both Android tuning and offline model generation. */
public final class ConfigSource {
  public static String source(Object config) throws IllegalAccessException {
    Class<?> type = config.getClass();
    StringBuilder s =
        new StringBuilder(
            "package org.firstinspires.ftc.teamcode.config;\n\npublic class "
                + type.getSimpleName()
                + " {\n");
    java.lang.reflect.Field[] fields = type.getFields();
    Arrays.sort(fields, Comparator.comparing(java.lang.reflect.Field::getName));
    for (java.lang.reflect.Field f : fields) {
      if (Modifier.isFinal(f.getModifiers())) continue;
      s.append(" public ")
          .append(Modifier.isStatic(f.getModifiers()) ? "static " : "")
          .append(f.getType().getSimpleName())
          .append(" ")
          .append(f.getName())
          .append(" = ")
          .append(literal(f.get(config)))
          .append(";\n");
    }
    return s.append("}\n").toString();
  }

  private static String literal(Object value) {
    if (value instanceof String)
      return "\"" + ((String) value).replace("\\", "\\\\").replace("\"", "\\\"") + "\"";
    if (value.getClass().isArray()) {
      StringJoiner j = new StringJoiner(", ", "{", "}");
      for (int i = 0; i < Array.getLength(value); i++) j.add(literal(Array.get(value, i)));
      return j.toString();
    }
    if (value instanceof Double && !Double.isFinite((Double) value))
      throw new IllegalArgumentException("Nonfinite configuration");
    return value.toString();
  }
}
