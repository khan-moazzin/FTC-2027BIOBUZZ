package org.firstinspires.ftc.teamcode.lib.calibration;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/** Complete Java source, kept separate from running configuration until copied and rebuilt. */
public final class ConfigExport {
  public static String source(Object config) throws IllegalAccessException {
    return ConfigSource.source(config);
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
