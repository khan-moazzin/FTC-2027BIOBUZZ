package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.*;

import java.io.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.teamcode.lib.logging.*;
import org.junit.Test;

public class LoggingTest {
  private static LogFrame frame(long time) {
    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Test/Number", 42.5);
    values.put("Test/Boolean", true);
    values.put("Test/Reason", "Rejected, \"pose\"\nretry");
    return new LogFrame(time, values);
  }

  @Test
  public void csvHasSecondsTypesEscapingAndDrainsAtStop() throws Exception {
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    AsyncCsvLog log = new AsyncCsvLog(bytes, 1000000000L, 8, 100000, null, 1);
    log.offer(frame(1250000000L));
    log.offer(frame(1500000000L));
    log.close();
    String csv = bytes.toString("UTF-8");
    assertTrue(csv.startsWith("Timestamp,Test/Number,Test/Boolean,Test/Reason\n"));
    String[] rows = csv.split("\n");
    assertEquals(3, rows.length);
    String[] cells = rows[1].split(",");
    assertEquals(4, cells.length);
    assertEquals("0.25", cells[0]);
    assertEquals("42.5", cells[1]);
    assertEquals("true", cells[2]);
    assertEquals(
        "Rejected, \"pose\"\nretry", new com.google.gson.Gson().fromJson(cells[3], String.class));
    assertEquals(2, log.written());
    assertEquals(0, log.dropped());
    assertEquals("Closed", log.status());
    // Retained synthetic fixture for importer smoke testing, never represented as robot data.
    File dir = new File("build/logging-fixtures");
    dir.mkdirs();
    try (OutputStream out = new FileOutputStream(new File(dir, "synthetic-ascope.csv"))) {
      out.write(bytes.toByteArray());
    }
  }

  @Test
  public void snapshotsDoNotChangeAndInvalidNumbersAreExplicit() {
    Map<String, Object> values = new LinkedHashMap<>();
    LogFrame.number(values, "Voltage", Double.NaN);
    LogFrame f = new LogFrame(0, values);
    values.put("Voltage", 12.0);
    assertEquals(0.0, (Double) f.values.get("Voltage"), 0);
    assertEquals(false, f.values.get("Voltage/Valid"));
    try {
      f.values.put("Voltage", 99);
      fail();
    } catch (UnsupportedOperationException expected) {
    }
  }

  @Test
  public void slowStorageHasBoundedQueueAndDropsWithoutWaiting() throws Exception {
    CountDownLatch writing = new CountDownLatch(1), release = new CountDownLatch(1);
    OutputStream slow =
        new OutputStream() {
          @Override
          public void write(int b) {}

          @Override
          public void write(byte[] b) throws IOException {
            writing.countDown();
            try {
              if (!release.await(3, TimeUnit.SECONDS)) throw new IOException("Test timeout");
            } catch (InterruptedException e) {
              throw new IOException(e);
            }
          }
        };
    AsyncCsvLog log = new AsyncCsvLog(slow, 0, 2, 100000, null, 1);
    try {
      log.offer(frame(1));
      assertTrue(writing.await(1, TimeUnit.SECONDS));
      for (int i = 2; i <= 101; i++) log.offer(frame(i));
      assertEquals(2, log.queued());
      assertEquals(98, log.dropped());
    } finally {
      release.countDown();
      log.close();
    }
    assertEquals(3, log.written());
  }

  @Test
  public void fileFailureDoesNotKillLiveStream() {
    AtomicInteger live = new AtomicInteger();
    OutputStream broken =
        new OutputStream() {
          @Override
          public void write(int b) throws IOException {
            throw new IOException("disk full");
          }
        };
    AsyncCsvLog log = new AsyncCsvLog(broken, 0, 8, 100000, f -> live.incrementAndGet(), 1);
    log.offer(frame(1));
    log.offer(frame(2));
    log.close();
    assertTrue(log.status().contains("disk full"));
    assertEquals(2, live.get());
    assertEquals(2, log.fileDropped());
  }

  @Test
  public void liveFailureDoesNotKillFileRecording() {
    AsyncCsvLog log =
        new AsyncCsvLog(
            new ByteArrayOutputStream(),
            0,
            8,
            100000,
            f -> {
              throw new IllegalStateException("dashboard offline");
            },
            1);
    log.offer(frame(1));
    log.offer(frame(2));
    log.close();
    assertEquals(2, log.written());
    assertTrue(log.liveStatus().contains("dashboard offline"));
  }

  @Test
  public void fileLimitNeverWritesPartialRowAndLiveIsThrottled() {
    AtomicInteger live = new AtomicInteger();
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    AsyncCsvLog log = new AsyncCsvLog(bytes, 0, 8, 1, f -> live.incrementAndGet(), 100);
    log.offer(frame(0));
    log.offer(frame(50));
    log.offer(frame(100));
    log.close();
    assertEquals(0, bytes.size());
    assertEquals(3, log.fileDropped());
    assertEquals(2, live.get());
    assertTrue(log.status().contains("limit"));
  }

  @Test
  public void backwardTimestampIsDroppedAndSchemaChangeIsReported() {
    AsyncCsvLog log = new AsyncCsvLog(new ByteArrayOutputStream(), 0, 8, 100000, null, 1);
    log.offer(frame(10));
    log.offer(frame(5));
    Map<String, Object> changed = new LinkedHashMap<>();
    changed.put("different", 1);
    log.offer(new LogFrame(20, changed));
    log.close();
    assertEquals(1, log.written());
    assertEquals(1, log.dropped());
    assertTrue(log.status().contains("schema changed"));
    log.offer(frame(30));
    assertEquals(2, log.dropped());
  }

  @Test
  public void closeIsIdempotentAndRejectsLaterSamples() {
    AsyncCsvLog log = new AsyncCsvLog(new ByteArrayOutputStream(), 0, 8, 100000, null, 1);
    log.offer(frame(1));
    log.close();
    log.close();
    log.offer(frame(2));
    assertEquals(1, log.written());
    assertEquals(1, log.dropped());
  }
}
