package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Aiming;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.planners.HiveTracker;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(
        name = "Shooter Calibration",
        group = "Tuning"
)
public class ShooterCalibration extends OpMode {

    private static final double DRIVE_ROTATION_SCALE = 0.6;
    private static final double DRIVE_DEADBAND = 0.05;

    private static final double COARSE_SPEED_STEP = 2.0;
    private static final double FINE_SPEED_STEP = 0.5;
    private static final double COARSE_HOOD_STEP_DEGREES = 1.0;
    private static final double FINE_HOOD_STEP_DEGREES = 0.25;
    private static final double COARSE_SERVO_STEP = 0.01;
    private static final double FINE_SERVO_STEP = 0.0025;

    private enum ServoTarget {
        HOOD,
        INDEXER_RETRACTED,
        INDEXER_DEPLOYED
    }

    private Robot robot;
    private ServoTarget selectedServo = ServoTarget.HOOD;

    private double flywheelTargetPercent = 0.0;
    private double hoodTargetDegrees = HOOD_STOW_DEGREES;
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
        double speedStep = fineAdjustment ? FINE_SPEED_STEP : COARSE_SPEED_STEP;
        double hoodStep = fineAdjustment
                ? FINE_HOOD_STEP_DEGREES : COARSE_HOOD_STEP_DEGREES;
        double servoStep = fineAdjustment ? FINE_SERVO_STEP : COARSE_SERVO_STEP;

        if (gamepad2.dpadUpWasPressed()) {
            flywheelTargetPercent = clamp(
                    flywheelTargetPercent + speedStep, 0.0, FLYWHEEL_MAX_PERCENT);
        }
        if (gamepad2.dpadDownWasPressed()) {
            flywheelTargetPercent = clamp(
                    flywheelTargetPercent - speedStep, 0.0, FLYWHEEL_MAX_PERCENT);
        }
        if (gamepad2.dpadRightWasPressed()) adjustSelected(hoodStep, servoStep);
        if (gamepad2.dpadLeftWasPressed()) adjustSelected(-hoodStep, -servoStep);

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

        robot.hood.setAngle(hoodTargetDegrees);
        robot.flywheel.setSpeed(flywheelEnabled ? flywheelTargetPercent : 0.0);
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

    private void adjustSelected(double hoodAmount, double servoAmount) {
        switch (selectedServo) {
            case HOOD:
                hoodTargetDegrees = clamp(
                        hoodTargetDegrees + hoodAmount, 0.0, HOOD_MAX_DEGREES);
                break;
            case INDEXER_RETRACTED:
                indexerRetracted = clamp(indexerRetracted + servoAmount, 0.0, 1.0);
                break;
            case INDEXER_DEPLOYED:
                indexerDeployed = clamp(indexerDeployed + servoAmount, 0.0, 1.0);
                break;
        }
    }

    private void selectNextServo() {
        ServoTarget[] targets = ServoTarget.values();
        selectedServo = targets[(selectedServo.ordinal() + 1) % targets.length];
    }

    private void emergencyStop() {
        flywheelEnabled = false;
        hoodTargetDegrees = HOOD_STOW_DEGREES;
    }

    private void sendCalibrationTelemetry(boolean manualFeed) {
        telemetry.addLine("GAMEPAD 2");
        telemetry.addLine("dpad UP/DOWN: flywheel % | dpad LEFT/RIGHT: selected value");
        telemetry.addLine("A: flywheel | B: stop | X: fine | Y: select servo");
        telemetry.addLine("Hold RIGHT BUMPER: intake + deploy indexer (flywheel must be enabled)");
        telemetry.addData("Adjustment", fineAdjustment ? "FINE" : "COARSE");
        telemetry.addData("Selected Servo", selectedServo);
        telemetry.addData("Flywheel Enabled", flywheelEnabled);
        telemetry.addData("Flywheel Target %", flywheelTargetPercent);
        telemetry.addData("Flywheel Measured RPM", robot.flywheel.getRpm());
        telemetry.addData("Hood Target deg", hoodTargetDegrees);
        telemetry.addData("Indexer Retracted", indexerRetracted);
        telemetry.addData("Indexer Deployed", indexerDeployed);
        telemetry.addData("Manual Feed", manualFeed);
        telemetry.addData("Selected CELL Distance in",
                Double.isFinite(shotDistanceInches)
                        ? String.format(Locale.US, "%.2f", shotDistanceInches) : "--");
        telemetry.addData("Shot Map Distance Entry",
                Double.isFinite(shotDistanceInches)
                        ? String.format(Locale.US, "%.2f", shotDistanceInches) : "--");
        telemetry.addData("HOOD_ANGLE_MAP value",
                String.format(Locale.US, "%.2f", hoodTargetDegrees));
        telemetry.addData("FLYWHEEL_SPEED_MAP value",
                String.format(Locale.US, "%.1f", flywheelTargetPercent));
        telemetry.addLine("Copy successful distance/hood/speed points into ShooterConstants.");
    }

    private static double deadband(double value) {
        return Math.abs(value) < DRIVE_DEADBAND ? 0.0 : value;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
