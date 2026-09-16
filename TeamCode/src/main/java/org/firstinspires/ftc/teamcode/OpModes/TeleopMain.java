package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Constants.HOOD_STOW;
import static org.firstinspires.ftc.teamcode.Constants.TURRET_CENTER;

@TeleOp(name = "TeleopMain", group = "Teleop")
public class TeleopMain extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private Robot mRobot;

    private static final double YAW_SCALE = 0.6;
    private static final double TRIGGER = 0.2;
    private static final double DEADBAND = 0.05;

    /** Bringup value until the shot map exists. */
    private static final double TEST_RPM = 3000.0;

    /** Servo units per second while a jog button is held. */
    private static final double TURRET_JOG = 0.25;
    private static final double HOOD_JOG = 0.25;

    private boolean lastIntake = false;
    private boolean lastOuttake = false;
    private boolean lastFlywheel = false;
    private boolean lastReset = false;

    @Override
    public void init() {
        // The Scheduler is static and survives OpMode restarts.
        Scheduler.reset();

        mRobot = new Robot();
        mRobot.init(hardwareMap, telemetry);

        // Robot-oriented until the Pinpoint is tuned.
        mRobot.drive.toggleRobotOriented();
    }

    @Override
    public void start() {
        runtime.reset();
        loopTimer.reset();

        Scheduler.schedule(mRobot.drive.teleopDrive(
                () -> -deadband(gamepad1.left_stick_y),
                () -> -deadband(gamepad1.left_stick_x),
                () -> -deadband(gamepad1.right_stick_x) * YAW_SCALE
        ));
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ================= DRIVER — GAMEPAD 1 =================
        boolean intakeHeld = gamepad1.left_trigger > TRIGGER;
        if (intakeHeld && !lastIntake) {
            Scheduler.schedule(mRobot.intake.intake()
                    .until(() -> gamepad1.left_trigger <= TRIGGER));
        }
        lastIntake = intakeHeld;

        boolean outtakeHeld = gamepad1.left_bumper;
        if (outtakeHeld && !lastOuttake) {
            Scheduler.schedule(mRobot.intake.outtake()
                    .until(() -> !gamepad1.left_bumper));
        }
        lastOuttake = outtakeHeld;

        boolean reset = gamepad1.start;
        if (reset && !lastReset) mRobot.drive.resetHeading();
        lastReset = reset;

        // ================= OPERATOR — GAMEPAD 2 =================
        boolean flywheelHeld = gamepad2.right_trigger > TRIGGER;
        if (flywheelHeld && !lastFlywheel) {
            Scheduler.schedule(mRobot.flywheel.spin(TEST_RPM)
                    .until(() -> gamepad2.right_trigger <= TRIGGER));
        }
        lastFlywheel = flywheelHeld;

        // Servo jogs are per-second so they feel the same at any loop rate.
        double turretJog = -deadband(gamepad2.left_stick_x);
        if (turretJog != 0.0) {
            mRobot.turret.setPosition(mRobot.turret.getPosition() + turretJog * TURRET_JOG * dt);
        }
        if (gamepad2.b) mRobot.turret.setPosition(TURRET_CENTER);

        double hoodJog = 0.0;
        if (gamepad2.dpad_up) hoodJog += 1.0;
        if (gamepad2.dpad_down) hoodJog -= 1.0;
        if (hoodJog != 0.0) {
            mRobot.hood.setPosition(mRobot.hood.getPosition() + hoodJog * HOOD_JOG * dt);
        }
        if (gamepad2.a) mRobot.hood.setPosition(HOOD_STOW);

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
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}