package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Aiming;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.HiveTracker;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
        name = "Shooter Calibration",
        group = "Tuning"
)
public class ShooterCalibration extends OpMode {

    private static final double DRIVE_ROTATION_SCALE = 0.6;
    private static final double DRIVE_DEADBAND = 0.05;

    private static final double COARSE_RPM_STEP = 100.0;
    private static final double FINE_RPM_STEP = 25.0;
    private static final double COARSE_SERVO_STEP = 0.01;
    private static final double FINE_SERVO_STEP = 0.0025;

    private enum ServoTarget {
        HOOD,
        INDEXER_RETRACTED,
        INDEXER_DEPLOYED
    }

    private Robot robot;
    private ServoTarget selectedServo = ServoTarget.HOOD;

    private double flywheelTargetRpm = 0.0;
    private double hoodTarget = HOOD_STOW;
    private double indexerRetracted = INDEXER_RETRACTED;
    private double indexerDeployed = INDEXER_DEPLOYED;

    private boolean flywheelEnabled = false;
    private boolean fineAdjustment = false;
    private double shotDistanceInches = Double.NaN;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot();
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) Constants.ALLIANCE = Constants.Alliance.BLUE;
        if (gamepad1.dpad_right) Constants.ALLIANCE = Constants.Alliance.RED;

        telemetry.addData("Alliance", Constants.ALLIANCE);
        telemetry.addLine("Gamepad 1 dpad: LEFT = BLUE, RIGHT = RED");
        telemetry.update();
    }

    @Override
    public void start() {
        Scheduler.schedule(robot.drive.teleopDrive(
                () -> -deadband(gamepad1.left_stick_y),
                () -> -deadband(gamepad1.left_stick_x),
                () -> -deadband(gamepad1.right_stick_x) * DRIVE_ROTATION_SCALE
        ));
    }

    @Override
    public void loop() {
        double rpmStep = fineAdjustment ? FINE_RPM_STEP : COARSE_RPM_STEP;
        double servoStep = fineAdjustment ? FINE_SERVO_STEP : COARSE_SERVO_STEP;

        if (gamepad2.dpadUpWasPressed()) {
            flywheelTargetRpm = clamp(flywheelTargetRpm + rpmStep, 0.0, FLYWHEEL_MAX_RPM);
        }
        if (gamepad2.dpadDownWasPressed()) {
            flywheelTargetRpm = clamp(flywheelTargetRpm - rpmStep, 0.0, FLYWHEEL_MAX_RPM);
        }
        if (gamepad2.dpadRightWasPressed()) adjustSelectedServo(servoStep);
        if (gamepad2.dpadLeftWasPressed()) adjustSelectedServo(-servoStep);

        if (gamepad2.aWasPressed()) flywheelEnabled = !flywheelEnabled;
        if (gamepad2.xWasPressed()) fineAdjustment = !fineAdjustment;
        if (gamepad2.yWasPressed()) selectNextServo();
        if (gamepad2.bWasPressed()) emergencyStop();

        boolean manualFeed = flywheelEnabled && gamepad2.right_bumper;

        HiveTracker.Target target = robot.hiveTracker.target(Constants.ALLIANCE);
        if (target != null) {
            shotDistanceInches = Aiming.distanceFromTurret(
                    robot.drive.getPose(), target.x, target.y);
            robot.turret.setAngle(Aiming.bearingFromTurret(
                    robot.drive.getPose(), target.x, target.y));
        } else {
            shotDistanceInches = Double.NaN;
        }

        robot.hood.setPosition(hoodTarget);
        robot.flywheel.setTargetRpm(flywheelEnabled ? flywheelTargetRpm : 0.0);
        robot.indexer.setPosition(manualFeed ? indexerDeployed : indexerRetracted);
        robot.intake.setPower(manualFeed ? INTAKE : INTAKE_IDLE);

        Scheduler.execute();
        robot.update();
        sendCalibrationTelemetry(manualFeed);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (robot != null) robot.stop();
    }

    private void adjustSelectedServo(double amount) {
        switch (selectedServo) {
            case HOOD:
                hoodTarget = clamp(hoodTarget + amount, HOOD_MIN, HOOD_MAX);
                break;
            case INDEXER_RETRACTED:
                indexerRetracted = clamp(indexerRetracted + amount, 0.0, 1.0);
                break;
            case INDEXER_DEPLOYED:
                indexerDeployed = clamp(indexerDeployed + amount, 0.0, 1.0);
                break;
        }
    }

    private void selectNextServo() {
        ServoTarget[] targets = ServoTarget.values();
        selectedServo = targets[(selectedServo.ordinal() + 1) % targets.length];
    }

    private void emergencyStop() {
        flywheelEnabled = false;
        hoodTarget = HOOD_STOW;
    }

    private void sendCalibrationTelemetry(boolean manualFeed) {
        telemetry.addLine("GAMEPAD 2");
        telemetry.addLine("dpad UP/DOWN: RPM | dpad LEFT/RIGHT: selected servo");
        telemetry.addLine("A: flywheel | B: stop | X: fine | Y: select servo");
        telemetry.addLine("Hold RIGHT BUMPER: intake + deploy indexer (flywheel must be enabled)");
        telemetry.addData("Adjustment", fineAdjustment ? "FINE" : "COARSE");
        telemetry.addData("Selected Servo", selectedServo);
        telemetry.addData("Flywheel Enabled", flywheelEnabled);
        telemetry.addData("Flywheel Target RPM", flywheelTargetRpm);
        telemetry.addData("Hood Target Position", hoodTarget);
        telemetry.addData("Indexer Retracted", indexerRetracted);
        telemetry.addData("Indexer Deployed", indexerDeployed);
        telemetry.addData("Manual Feed", manualFeed);
        telemetry.addData("Selected CELL Distance in",
                Double.isFinite(shotDistanceInches)
                        ? String.format(Locale.US, "%.2f", shotDistanceInches) : "--");
        telemetry.addData("Shot Map Distance Entry",
                Double.isFinite(shotDistanceInches)
                        ? String.format(Locale.US, "%.2f", shotDistanceInches) : "--");
        telemetry.addData("Shot Map Hood Entry", String.format(Locale.US, "%.4f", hoodTarget));
        telemetry.addData("Shot Map RPM Entry", String.format(Locale.US, "%.0f", flywheelTargetRpm));
        telemetry.addLine("Copy successful distance/hood/RPM points into ShooterConstants.");
    }

    private static double deadband(double value) {
        return Math.abs(value) < DRIVE_DEADBAND ? 0.0 : value;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
