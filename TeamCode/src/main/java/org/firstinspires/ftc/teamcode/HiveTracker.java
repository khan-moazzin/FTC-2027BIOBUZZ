package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

/**
 * Tracks the selected alliance HIVE from the plane normals of its AprilTags.
 *
 * Limelight camera space is +X right, +Y down, +Z forward. This class converts
 * that normal into Pedro field space, where the HIVE rotates about +field X.
 * A shot may be released early only when a fresh, consistently directed angle
 * rate predicts that the selected CELL will reach its stable stop before the
 * configured feed lead time expires.
 */
public final class HiveTracker {

    public enum CellSide {
        AUDIENCE,
        SCORING
    }

    public enum State {
        UNKNOWN,
        AUDIENCE_UP,
        SCORING_UP,
        MOVING
    }

    public static final class Target {
        public final double x;
        public final double y;
        public final CellSide cell;

        private Target(double x, double y, CellSide cell) {
            this.x = x;
            this.y = y;
            this.cell = cell;
        }
    }

    private long lastFrameTimestampMillis = Long.MIN_VALUE;
    private long lastFrameReceiptNanos;
    private long lastCellSelectionNanos;
    private double frameAgeAtReceiptMs = Double.POSITIVE_INFINITY;

    private double angleDegrees = Double.NaN;
    private double lastMeasuredAngleDegrees = Double.NaN;
    private double rateDegreesPerSecond = Double.NaN;
    private int rateSamples;
    private int tagsUsed;
    private CellSide selectedCell;
    private String status = "no HIVE frame";

    public void update(
            Limelight limelight,
            Pose robotPose,
            double turretAngleDegrees,
            Constants.Alliance alliance
    ) {
        long frameTimestamp = limelight.frameTimestampMillis();
        if (frameTimestamp == Long.MIN_VALUE || frameTimestamp == lastFrameTimestampMillis) return;
        lastFrameTimestampMillis = frameTimestamp;

        double frameAgeMs = limelight.frameAgeMs();
        if (!limelight.hasTarget() || !Double.isFinite(frameAgeMs)
                || frameAgeMs > HIVE_MAX_VISION_AGE_MS) {
            status = "new Limelight frame is invalid/stale";
            return;
        }

        List<Double> measurements = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducial : limelight.fiducials()) {
            int id = fiducial.getFiducialId();
            if (!isAllianceHiveTag(id, alliance)) continue;

            Double measurement = angleFromTag(
                    fiducial.getTargetPoseCameraSpace(),
                    robotPose.heading(),
                    turretAngleDegrees);
            if (measurement != null) measurements.add(measurement);
        }

        if (measurements.isEmpty()) {
            tagsUsed = 0;
            status = "no usable alliance HIVE tag normals";
            return;
        }

        Collections.sort(measurements);
        double measuredAngle = median(measurements);
        long now = System.nanoTime();

        if (Double.isFinite(lastMeasuredAngleDegrees) && lastFrameReceiptNanos != 0L) {
            double dt = (now - lastFrameReceiptNanos) / 1_000_000_000.0;
            if (dt >= 0.005 && dt <= 0.50) {
                double rawRate = (measuredAngle - lastMeasuredAngleDegrees) / dt;
                rateDegreesPerSecond = Double.isFinite(rateDegreesPerSecond)
                        ? lerp(rateDegreesPerSecond, rawRate, HIVE_RATE_FILTER)
                        : rawRate;
                rateSamples++;
            } else {
                rateDegreesPerSecond = Double.NaN;
                rateSamples = 0;
            }
        }

        angleDegrees = Double.isFinite(angleDegrees)
                ? lerp(angleDegrees, measuredAngle, HIVE_ANGLE_FILTER)
                : measuredAngle;
        lastMeasuredAngleDegrees = measuredAngle;
        lastFrameReceiptNanos = now;
        frameAgeAtReceiptMs = frameAgeMs;
        tagsUsed = measurements.size();

        CellSide observedCell = chooseCell();
        if (observedCell != null) {
            selectedCell = observedCell;
            lastCellSelectionNanos = now;
        }
        status = "tracking";
    }

    /**
     * Extracts the tag's +Z plane normal from Limelight's target-to-camera
     * roll/pitch/yaw transform and returns the HIVE angle in degrees.
     */
    private static Double angleFromTag(
            Pose3D targetPoseCameraSpace,
            double robotHeadingRadians,
            double turretAngleDegrees
    ) {
        if (targetPoseCameraSpace == null) return null;

        YawPitchRollAngles orientation = targetPoseCameraSpace.getOrientation();
        double roll = orientation.getRoll(AngleUnit.RADIANS);
        double pitch = orientation.getPitch(AngleUnit.RADIANS);
        double yaw = orientation.getYaw(AngleUnit.RADIANS);
        if (!allFinite(roll, pitch, yaw)) return null;

        // Third column of Rz(yaw) * Ry(pitch) * Rx(roll).
        double sinRoll = Math.sin(roll), cosRoll = Math.cos(roll);
        double sinPitch = Math.sin(pitch), cosPitch = Math.cos(pitch);
        double sinYaw = Math.sin(yaw), cosYaw = Math.cos(yaw);
        double cameraX = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
        double cameraY = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
        double cameraZ = cosPitch * cosRoll;

        // Level/upright camera space -> turret frame (+forward, +left, +up).
        double forward = cameraZ;
        double left = -cameraX;
        double up = -cameraY;

        // Apply the measured camera mount roll and pitch. Positive pitch points
        // the optical axis upward; roll is right-handed about optical forward.
        double mountRoll = Math.toRadians(LIMELIGHT_ROLL_DEGREES);
        double rollLeft = Math.cos(mountRoll) * left - Math.sin(mountRoll) * up;
        double rollUp = Math.sin(mountRoll) * left + Math.cos(mountRoll) * up;

        double mountPitch = Math.toRadians(LIMELIGHT_PITCH_DEGREES);
        double pitchedForward = Math.cos(mountPitch) * forward
                - Math.sin(mountPitch) * rollUp;
        double pitchedUp = Math.sin(mountPitch) * forward
                + Math.cos(mountPitch) * rollUp;

        double cameraFieldHeading = robotHeadingRadians + Math.toRadians(turretAngleDegrees);
        double fieldX = Math.cos(cameraFieldHeading) * pitchedForward
                - Math.sin(cameraFieldHeading) * rollLeft;
        double fieldY = Math.sin(cameraFieldHeading) * pitchedForward
                + Math.cos(cameraFieldHeading) * rollLeft;
        double fieldZ = pitchedUp;

        double norm = Math.sqrt(fieldX * fieldX + fieldY * fieldY + fieldZ * fieldZ);
        if (!Double.isFinite(norm) || norm < 1e-9) return null;
        fieldX /= norm;
        fieldY /= norm;
        fieldZ /= norm;

        // Tag normals are direction-ambiguous. Both CELL tag planes describe
        // the same HIVE plane after the normal is selected downward.
        if (fieldZ > 0.0) {
            fieldX = -fieldX;
            fieldY = -fieldY;
            fieldZ = -fieldZ;
        }
        if (Math.abs(fieldX) > HIVE_MAX_NORMAL_X) return null;

        double angle = Math.toDegrees(Math.atan2(fieldY, -fieldZ));
        double maxTrackAngle = HIVE_STABLE_ANGLE_DEGREES + 10.0;
        return Math.abs(angle) <= maxTrackAngle + 5.0
                ? clamp(angle, -maxTrackAngle, maxTrackAngle)
                : null;
    }

    public boolean hasFreshVision() {
        return Double.isFinite(angleDegrees) && visionAgeMs() <= HIVE_MAX_VISION_AGE_MS;
    }

    public double visionAgeMs() {
        if (lastFrameReceiptNanos == 0L) return Double.POSITIVE_INFINITY;
        return frameAgeAtReceiptMs
                + (System.nanoTime() - lastFrameReceiptNanos) / 1_000_000.0;
    }

    public State state() {
        if (!hasFreshVision()) return State.UNKNOWN;
        if (Math.abs(angleDegrees - HIVE_STABLE_ANGLE_DEGREES)
                <= HIVE_STABLE_TOLERANCE_DEGREES) {
            return State.SCORING_UP;
        }
        if (Math.abs(angleDegrees + HIVE_STABLE_ANGLE_DEGREES)
                <= HIVE_STABLE_TOLERANCE_DEGREES) {
            return State.AUDIENCE_UP;
        }
        return State.MOVING;
    }

    /** True only for a stable HIVE or a validated early-release prediction. */
    public boolean inFeedWindow() {
        State state = state();
        if (state == State.SCORING_UP || state == State.AUDIENCE_UP) return true;
        if (state != State.MOVING || selectedCell == null || rateSamples < 2
                || !Double.isFinite(rateDegreesPerSecond)
                || Math.abs(rateDegreesPerSecond) < HIVE_MIN_RATE_DEG_S) {
            return false;
        }

        double seconds = timeToStableSeconds();
        return Double.isFinite(seconds)
                && seconds >= 0.0
                && seconds <= HIVE_EARLY_FEED_LEAD_SECONDS;
    }

    public boolean isEarlyFeedWindow() {
        return state() == State.MOVING && inFeedWindow();
    }

    public double timeToStableSeconds() {
        if (selectedCell == null || !Double.isFinite(rateDegreesPerSecond)
                || Math.abs(rateDegreesPerSecond) < HIVE_MIN_RATE_DEG_S) {
            return Double.NaN;
        }
        double targetAngle = selectedCell == CellSide.SCORING
                ? HIVE_STABLE_ANGLE_DEGREES
                : -HIVE_STABLE_ANGLE_DEGREES;
        double seconds = (targetAngle - angleDegrees) / rateDegreesPerSecond;
        return seconds >= 0.0 ? seconds : Double.NaN;
    }

    /**
     * Returns the selected CELL's center at its stable stop. Selection is kept
     * for pose-only turret tracking after vision loss, but feed remains disabled.
     */
    public Target target(Constants.Alliance alliance) {
        if (selectedCell == null || lastCellSelectionNanos == 0L) return null;
        double selectionAge = (System.nanoTime() - lastCellSelectionNanos) / 1_000_000_000.0;
        if (selectionAge > HIVE_AIM_MEMORY_SECONDS) return null;

        double hiveX = alliance == Constants.Alliance.RED ? RED_HIVE_X : BLUE_HIVE_X;
        double hiveY = alliance == Constants.Alliance.RED ? RED_HIVE_Y : BLUE_HIVE_Y;
        double sideSign = selectedCell == CellSide.SCORING ? 1.0 : -1.0;
        double stableY = sideSign * CELL_TARGET_RADIUS_IN
                * Math.cos(Math.toRadians(HIVE_STABLE_ANGLE_DEGREES));
        return new Target(hiveX, hiveY + stableY, selectedCell);
    }

    public double angleDegrees() {
        return angleDegrees;
    }

    public double rateDegreesPerSecond() {
        return rateDegreesPerSecond;
    }

    public int tagsUsed() {
        return tagsUsed;
    }

    public CellSide selectedCell() {
        return selectedCell;
    }

    public String status() {
        return status;
    }

    private CellSide chooseCell() {
        if (Math.abs(angleDegrees - HIVE_STABLE_ANGLE_DEGREES)
                <= HIVE_STABLE_TOLERANCE_DEGREES) {
            return CellSide.SCORING;
        }
        if (Math.abs(angleDegrees + HIVE_STABLE_ANGLE_DEGREES)
                <= HIVE_STABLE_TOLERANCE_DEGREES) {
            return CellSide.AUDIENCE;
        }
        if (!Double.isFinite(rateDegreesPerSecond)
                || Math.abs(rateDegreesPerSecond) < HIVE_MIN_RATE_DEG_S) {
            return null;
        }
        return rateDegreesPerSecond > 0.0 ? CellSide.SCORING : CellSide.AUDIENCE;
    }

    private static boolean isAllianceHiveTag(int id, Constants.Alliance alliance) {
        return alliance == Constants.Alliance.RED
                ? id >= 30 && id <= 37
                : id >= 38 && id <= 45;
    }

    private static double median(List<Double> sorted) {
        int size = sorted.size();
        return size % 2 == 1
                ? sorted.get(size / 2)
                : 0.5 * (sorted.get(size / 2 - 1) + sorted.get(size / 2));
    }

    private static boolean allFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) return false;
        }
        return true;
    }

    private static double lerp(double a, double b, double alpha) {
        return a + clamp(alpha, 0.0, 1.0) * (b - a);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
