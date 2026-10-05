package org.firstinspires.ftc.teamcode.config;

/** Logging limits are captured at INIT. Changes take effect on the next OpMode. */
public final class LoggingConfig {
  public static boolean enabled = true;
  public static boolean live = true;
  public static int queueCapacity = 256;
  public static long maxFileBytes = 64L * 1024 * 1024;
  public static long minimumFreeBytes = 128L * 1024 * 1024;
  public static int livePeriodMs = 100;
}
