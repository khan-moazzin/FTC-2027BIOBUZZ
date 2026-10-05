package org.firstinspires.ftc.teamcode.lib.logging;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

/** AdvantageScope CSV Table writer. A slow sink drops samples instead of waiting in control. */
public final class AsyncCsvLog implements AutoCloseable {
  private final ArrayBlockingQueue<LogFrame> queue;
  private final OutputStream output;
  private final Consumer<LogFrame> live;
  private final long epoch, limit, livePeriod;
  private final Thread worker;
  private volatile boolean accepting = true, abort;
  private volatile String status = "Recording", liveStatus = "Ready";
  private volatile long bytes, written;
  private final AtomicLong dropped = new AtomicLong();
  private volatile long fileDropped;
  private List<String> keys;

  public AsyncCsvLog(
      OutputStream output,
      long epoch,
      int capacity,
      long limit,
      Consumer<LogFrame> live,
      long livePeriodNs) {
    if (capacity < 1 || limit < 1 || livePeriodNs < 1)
      throw new IllegalArgumentException("Invalid log limits");
    this.output = output;
    this.epoch = epoch;
    this.limit = limit;
    this.live = live;
    this.livePeriod = livePeriodNs;
    queue = new ArrayBlockingQueue<>(capacity);
    worker = new Thread(this::run, "BioBuzz-log");
    worker.setDaemon(true);
    worker.start();
  }

  /** Single producer on the OpMode thread; never waits for capacity or storage. */
  public void offer(LogFrame frame) {
    if (!accepting || !queue.offer(frame)) dropped.incrementAndGet();
  }

  private void run() {
    boolean disk = true, streaming = live != null;
    long lastLive = Long.MIN_VALUE, flushAt = System.nanoTime();
    long previous = epoch;
    try {
      if (live == null) liveStatus = "Disabled";
      while (!abort && (accepting || !queue.isEmpty())) {
        LogFrame frame = queue.poll(100, TimeUnit.MILLISECONDS);
        if (frame != null) {
          if (frame.time < previous) {
            dropped.incrementAndGet();
            continue;
          }
          previous = frame.time;
          long beforeWrite = written;
          if (disk) {
            try {
              byte[] row = encode(frame);
              if (bytes + row.length > limit) {
                disk = false;
                status = "File size limit reached";
              } else {
                output.write(row);
                bytes += row.length;
                written++;
              }
            } catch (IOException | RuntimeException e) {
              disk = false;
              status = "File error: " + e;
            }
          }
          if (written == beforeWrite) fileDropped++;
          if (streaming
              && !abort
              && (lastLive == Long.MIN_VALUE || frame.time - lastLive >= livePeriod)) {
            try {
              live.accept(frame);
              lastLive = frame.time;
            } catch (RuntimeException e) {
              streaming = false;
              liveStatus = "Live error: " + e;
            }
          }
        }
        if (disk && System.nanoTime() - flushAt >= 1000000000L) {
          try {
            output.flush();
          } catch (IOException e) {
            disk = false;
            status = "Flush error: " + e;
          }
          flushAt = System.nanoTime();
        }
      }
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    } finally {
      accepting = false;
      dropped.addAndGet(queue.size());
      queue.clear();
      try {
        output.close();
      } catch (IOException e) {
        status = "Close error: " + e;
      }
      if (status.equals("Recording")) status = abort ? "Close timed out" : "Closed";
    }
  }

  private byte[] encode(LogFrame frame) {
    StringBuilder b = new StringBuilder();
    if (keys == null) {
      keys = new ArrayList<>(frame.values.keySet());
      b.append("Timestamp");
      for (String key : keys) {
        if (key.contains(",") || key.contains("\n") || key.contains("\r") || key.contains("\""))
          throw new IllegalArgumentException("Unsafe CSV field name");
        b.append(',').append(key);
      }
      b.append('\n');
    } else if (!new ArrayList<>(frame.values.keySet()).equals(keys)) {
      throw new IllegalArgumentException("Log schema changed during run");
    }
    b.append((frame.time - epoch) * 1e-9);
    for (String key : keys) {
      Object value = frame.values.get(key);
      b.append(',').append(value instanceof String ? quote((String) value) : value);
    }
    b.append('\n');
    return b.toString().getBytes(StandardCharsets.UTF_8);
  }

  // AdvantageScope CSVDecoder splits on commas/newlines then JSON.parse()s each cell.
  // Use JSON escapes (including comma), not RFC 4180 doubled quotes or embedded newlines.
  private static String quote(String s) {
    StringBuilder b = new StringBuilder("\"");
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      switch (c) {
        case '"':
          b.append("\\\"");
          break;
        case '\\':
          b.append("\\\\");
          break;
        case ',':
          b.append("\\u002c");
          break;
        case '\n':
          b.append("\\n");
          break;
        case '\r':
          b.append("\\r");
          break;
        case '\t':
          b.append("\\t");
          break;
        default:
          if (c < 32) b.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
          else b.append(c);
      }
    }
    return b.append('"').toString();
  }

  public String status() {
    return status;
  }

  public String liveStatus() {
    return liveStatus;
  }

  public long dropped() {
    return dropped.get();
  }

  public long fileDropped() {
    return fileDropped;
  }

  public long written() {
    return written;
  }

  public long bytes() {
    return bytes;
  }

  public int queued() {
    return queue.size();
  }

  /** Called after actuator shutdown. Drain is bounded; no unbounded STOP wait. */
  @Override
  public void close() {
    accepting = false;
    try {
      worker.join(1500);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    if (worker.isAlive()) {
      status = "Close timed out";
      abort = true;
      worker.interrupt();
    }
  }
}
