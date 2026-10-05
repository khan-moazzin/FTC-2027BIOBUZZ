package org.firstinspires.ftc.teamcode.lib.vision;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.config.VisionConfig;
import org.firstinspires.ftc.teamcode.lib.field.Field;
import org.firstinspires.ftc.teamcode.lib.math.Vec3;

/** Raw per-tag camera-space data only: static botpose cannot describe moving HIVE tags. */
public final class LimelightSource implements AutoCloseable {
  public static final class Frame {
    public final long timestamp;
    public final List<TagObservation> tags;

    Frame(long t, List<TagObservation> v) {
      timestamp = t;
      tags = Collections.unmodifiableList(v);
    }
  }

  private final Limelight3A camera;
  private final VisionConfig c;
  private double last = Double.NaN;
  public String status = "Waiting for frame";

  public LimelightSource(HardwareMap hw, VisionConfig c) {
    this.c = c;
    camera = hw.get(Limelight3A.class, c.camera);
    camera.pipelineSwitch(c.pipeline);
    camera.start();
  }

  public Frame poll(long now) {
    LLResult r = camera.getLatestResult();
    if (r == null || !r.isValid() || r.getPipelineIndex() != c.pipeline) return null;
    double stamp = r.getTimestamp();
    if (!Double.isFinite(stamp) || stamp == last) return null;
    last = stamp;
    double age =
        (r.getStaleness() + r.getCaptureLatency() + r.getTargetingLatency() + r.getParseLatency())
                * .001
            + c.extraLatency;
    if (!Double.isFinite(age) || age < 0 || age > c.maxFrameAge) {
      status = "Stale/invalid latency";
      return null;
    }
    List<TagObservation> tags = new ArrayList<>();
    Set<Integer> seen = new HashSet<>();
    for (LLResultTypes.FiducialResult tag : r.getFiducialResults()) {
      int id = tag.getFiducialId();
      if (Field.hive(id) == null || !seen.add(id) || tag.getTargetPoseCameraSpace() == null)
        continue;
      Position p = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
      Vec3 v = new Vec3(p.z, -p.x, -p.y);
      double range = v.norm();
      if (v.finite() && v.x > 0 && range >= c.minRange && range <= c.maxRange)
        tags.add(new TagObservation(id, v));
    }
    status = tags.size() + " valid tags";
    return new Frame(now - (long) (age * 1e9), tags);
  }

  @Override
  public void close() {
    camera.stop();
  }
}
