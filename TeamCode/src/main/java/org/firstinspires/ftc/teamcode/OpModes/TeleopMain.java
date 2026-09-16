package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Drive and intake only. Xbox and Logitech expose identical button names in the SDK,
 * so nothing here is controller-specific.
 */

@TeleOp(name = "TeleopMain", group = "Teleop")
public class TeleopMain extends OpMode {

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

        // Robot-oriented until the Pinpoint is tuned.
        mRobot.drive.toggleRobotOriented();
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

        // back, not start: the Driver Station uses start+A / start+B to bind gamepads
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
    }

    private static double deadband(double value) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }
}
