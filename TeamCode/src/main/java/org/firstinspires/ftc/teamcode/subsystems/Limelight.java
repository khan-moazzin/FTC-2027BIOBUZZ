package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.LIMELIGHT_PIPELINE;

/** Turret-mounted Limelight 3A running the HIVE AprilTag pipeline. */
public class Limelight {

    private final Limelight3A limelight;
    private LLResult result;

    public Limelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public long frameTimestampMillis() {
        return result == null ? Long.MIN_VALUE : result.getControlHubTimeStamp();
    }

    public double frameAgeMs() {
        if (result == null) return Double.POSITIVE_INFINITY;
        return Math.max(0.0, result.getStaleness())
                + result.getCaptureLatency()
                + result.getTargetingLatency();
    }

    public List<LLResultTypes.FiducialResult> fiducials() {
        if (!hasTarget()) return Collections.emptyList();
        return result.getFiducialResults();
    }
}
