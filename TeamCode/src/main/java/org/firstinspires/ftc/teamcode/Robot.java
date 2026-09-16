package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


public class Robot {

    public Drive drive;
    public Intake intake;
    public Turret turret;
    public Hood hood;
    public Flywheel flywheel;

    private Telemetry telemetry;

    public void init(HardwareMap hw, Telemetry tele) {
        this.telemetry = tele;

        drive = new Drive(hw);
        intake = new Intake(hw);
        turret = new Turret(hw);
        hood = new Hood(hw);
        flywheel = new Flywheel(hw);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void update() {
        drive.update();
        sendTelemetry();
    }

    public void sendTelemetry() {
        telemetry.addData("X", drive.getPose().x());
        telemetry.addData("Y", drive.getPose().y());
        telemetry.addData("Heading", Math.toDegrees(drive.getPose().heading()));
        telemetry.addData("Field Oriented", !drive.isRobotOriented());
        telemetry.addData("Intake", intake.getPower());
        telemetry.addData("Turret", turret.getPosition());
        telemetry.addData("Hood", hood.getPosition());
        telemetry.addData("Flywheel RPM", flywheel.getRpm());
        telemetry.addData("Flywheel Target", flywheel.getTargetRpm());
        telemetry.addData("Flywheel At Speed", flywheel.atSpeed());
    }
}