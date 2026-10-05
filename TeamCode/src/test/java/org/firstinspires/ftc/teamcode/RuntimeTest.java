package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import java.io.StringWriter;
import org.firstinspires.ftc.teamcode.lib.control.CachedVoltage;
import org.firstinspires.ftc.teamcode.lib.control.LoopTiming;
import org.junit.Test;

public class RuntimeTest {
  @Test
  public void voltageRefreshIsThrottledAndBadReadIsNotReused() {
    int[] reads = {0};
    double[] sample = {12};
    CachedVoltage voltage =
        new CachedVoltage(
            () -> {
              reads[0]++;
              return sample[0];
            });
    assertTrue(Double.isNaN(voltage.volts(0)));
    for (long t = 0; t < 1000000000L; t += 1000000L) voltage.read(t);
    assertEquals(5, reads[0]);
    assertEquals(12, voltage.volts(999000000L), 0);
    sample[0] = Double.NaN;
    voltage.read(1000000000L);
    assertTrue(Double.isNaN(voltage.volts(1000000000L)));
    sample[0] = 13;
    voltage.read(1200000000L);
    assertTrue(Double.isNaN(voltage.volts(1800000000L)));
    assertTrue(Double.isNaN(voltage.volts(0)));
  }

  @Test
  public void voltageCompensationPreservesVoltsAndClampsDuty() {
    assertEquals(6, CachedVoltage.duty(6, 12) * 12, 1e-9);
    assertEquals(6, CachedVoltage.duty(6, 10) * 10, 1e-9);
    assertEquals(1, CachedVoltage.duty(14, 12), 0);
    assertEquals(0, CachedVoltage.duty(6, Double.NaN), 0);
    assertEquals(0, CachedVoltage.duty(Double.NaN, 12), 0);
    assertEquals(0, CachedVoltage.duty(6, 0), 0);
  }

  @Test
  public void profilerIncludesTelemetryGapAndKeepsBoundedChronologicalCsv() throws Exception {
    LoopTiming timing = new LoopTiming();
    for (int i = 0; i < 600; i++) {
      long t = i * 20000000L;
      timing.start(t);
      timing.readDone(t + 2000000);
      timing.finish(t + 5000000, t + 10000000);
    }
    StringWriter writer = new StringWriter();
    timing.writeCsv(writer);
    String[] rows = writer.toString().split("\n");
    assertEquals(513, rows.length);
    assertEquals("1760000000,2.0,3.0,5.0,10.0,20.0", rows[1]);
    assertEquals("11980000000,2.0,3.0,5.0,10.0,", rows[512]);
    assertTrue(timing.summary().contains("work p50 10.0"));
    assertTrue(timing.summary().contains("period p50 20.0"));
  }
}
