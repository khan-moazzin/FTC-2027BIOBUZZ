package org.firstinspires.ftc.teamcode.calibration;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** Complete Java source, kept separate from running configuration until copied and rebuilt. */
public final class ConfigExport {
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

  public static File saveText(String name, String text) throws IOException {
    File dir = new File(AppUtil.FIRST_FOLDER, "biobuzz-calibration");
    if (!dir.exists() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
    File file = new File(dir, name);
    try (Writer w = new OutputStreamWriter(new FileOutputStream(file), "UTF-8")) {
      w.write(text);
    }
    return file;
  }

  public static File save(Object c) throws IOException, IllegalAccessException {
    File dir = new File(AppUtil.FIRST_FOLDER, "biobuzz-calibration");
    if (!dir.exists() && !dir.mkdirs()) throw new IOException("Cannot create " + dir);
    File file = new File(dir, c.getClass().getSimpleName() + ".java");
    try (Writer w = new OutputStreamWriter(new FileOutputStream(file), "UTF-8")) {
      w.write(source(c));
    }
    return file;
  }
}
