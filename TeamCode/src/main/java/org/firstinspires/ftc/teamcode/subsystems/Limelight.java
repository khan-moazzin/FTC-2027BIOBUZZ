package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Turret-mounted Limelight 3A, AprilTag pipeline 1.
 * The Limelight must be configured with an IDENTITY camera pose, so getBotpose()
 * reports the CAMERA in field space; robotPose() un-rotates it by the turret angle.
 */
public class Limelight {

    private static final double METERS_TO_INCHES = 39.3701;

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

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double tx() {
        return hasTarget() ? result.getTx() : 0.0;
    }

    public double ty() {
        return hasTarget() ? result.getTy() : 0.0;
    }

    public double latencyMs() {
        return hasTarget() ? result.getCaptureLatency() + result.getTargetingLatency() : 0.0;
    }

    public Pose3D cameraPose() {
        return hasTarget() ? result.getBotpose() : null;
    }

    /**
     * Robot pose from the turret-mounted camera: undo the turret rotation, then back
     * out the camera-to-turret and turret-to-center offsets.
     */
    public static Pose robotPose(Pose3D cameraPose, double turretAngleDegrees) {
        double camHeading = Math.toRadians(cameraPose.getOrientation().getYaw());
        double robotHeading = camHeading - Math.toRadians(turretAngleDegrees);

        double camX = cameraPose.getPosition().x * METERS_TO_INCHES;
        double camY = cameraPose.getPosition().y * METERS_TO_INCHES;

        double axisX = camX - Math.cos(camHeading) * LL_FORWARD_FROM_TURRET_IN;
        double axisY = camY - Math.sin(camHeading) * LL_FORWARD_FROM_TURRET_IN;

        double cos = Math.cos(robotHeading), sin = Math.sin(robotHeading);
        double robotX = axisX - (cos * TURRET_FORWARD_IN - sin * TURRET_LEFT_IN);
        double robotY = axisY - (sin * TURRET_FORWARD_IN + cos * TURRET_LEFT_IN);

        return new Pose(robotX, robotY, Math.atan2(Math.sin(robotHeading), Math.cos(robotHeading)));
    }
}
