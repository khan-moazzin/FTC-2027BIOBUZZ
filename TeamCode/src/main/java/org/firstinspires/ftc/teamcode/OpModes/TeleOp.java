package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleopMain", group = "Teleop")
public class TeleOp extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private Robot mRobot;

    private static final double rotationScale = 0.6;
    private static final double deadband = 0.05;
    private boolean lastReset = false;

    @Override
    public void init() {
        Scheduler.reset();
        mRobot = new Robot();
        mRobot.init(hardwareMap, telemetry);
    }

    /** Pick the alliance before match start. */
    @Override
    public void init_loop() {
        if (gamepad1.dpad_left)  Constants.ALLIANCE = Constants.Alliance.BLUE;
        if (gamepad1.dpad_right) Constants.ALLIANCE = Constants.Alliance.RED;

        telemetry.addData("ALLIANCE", Constants.ALLIANCE);
        telemetry.addLine("dpad LEFT = BLUE, dpad RIGHT = RED");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        Scheduler.schedule(mRobot.drive.teleopDrive(
                () -> -deadband(gamepad1.left_stick_y),
                () -> -deadband(gamepad1.left_stick_x),
                () -> -deadband(gamepad1.right_stick_x) * rotationScale
        ));
    }

    @Override
    public void loop() {

        // ================= DRIVER — GAMEPAD 1 =================
        if (gamepad1.leftTriggerWasPressed()) {
            Scheduler.schedule(mRobot.intake.intake()
                    .until(() -> !gamepad1.left_trigger_pressed));
        }

        if (gamepad1.leftBumperWasPressed()) {
            Scheduler.schedule(mRobot.intake.outtake()
                    .until(() -> !gamepad1.left_bumper));
        }

        if (gamepad1.rightBumperWasPressed()) {
            Scheduler.schedule(mRobot.shooting.shoot()
                    .until(() -> !gamepad1.right_bumper));
        }

        boolean reset = gamepad1.back;
        if (reset && !lastReset) mRobot.drive.resetHeading();
        lastReset = reset;

        // ================= SCHEDULER =================
        Scheduler.execute();
        mRobot.update();

        telemetry.addData("Runtime", runtime.seconds());
        telemetry.update();

    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (mRobot != null) mRobot.stop();
    }

    //Stick Drift Helper
    private static double deadband(double value) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }
}
