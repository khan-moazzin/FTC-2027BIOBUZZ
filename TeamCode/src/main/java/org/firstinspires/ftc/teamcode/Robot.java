package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


public class Robot {

    public Drive drive;
    public Intake intake;
    public Turret turret;
    public Hood hood;
    public Flywheel flywheel;
    public Indexer indexer;
    public Limelight limelight;
    public HiveTracker hiveTracker;
    public ShootingController shooting;

    private Telemetry telemetry;

    public void init(HardwareMap hw, Telemetry tele) {
        this.telemetry = tele;

        drive = new Drive(hw);
        intake = new Intake(hw);
        turret = new Turret(hw);
        hood = new Hood(hw);
        flywheel = new Flywheel(hw);
        indexer = new Indexer(hw);
        limelight = new Limelight(hw);
        hiveTracker = new HiveTracker();
        shooting = new ShootingController(
                drive, intake, turret, hood, flywheel, indexer, hiveTracker);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void update() {
        drive.update();
        limelight.update();
        hiveTracker.update(limelight, drive.getPose(), turret.getAngle(), Constants.ALLIANCE);
        sendTelemetry();
    }

    public void stop() {
        shooting.stop();
        limelight.stop();
    }

    public void sendTelemetry() {
        telemetry.addData("X", drive.getPose().x());
        telemetry.addData("Y", drive.getPose().y());
        telemetry.addData("Heading", Math.toDegrees(drive.getPose().heading()));
        telemetry.addData("Drive Mode", "Field Oriented");
        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Turret Target Degrees", turret.getAngle());
        telemetry.addData("Turret Target Position", turret.getPosition());
        telemetry.addData("Hood", hood.getPosition());
        telemetry.addData("Flywheel RPM", flywheel.getRpm());
        telemetry.addData("Flywheel Target", flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", flywheel.atSpeed());
        telemetry.addData("Indexer", indexer.getPosition());
        shooting.addTelemetry(telemetry);
    }
}
