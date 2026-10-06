package org.firstinspires.ftc.teamcode.planners;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Aiming;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.planners.ShooterConstants.*;

/** Owns the coordinated aim/spin/readiness/feed sequence while shoot is held. */
public final class ShootingController {

    private final Drive drive;
    private final Intake intake;
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;
    private final Indexer indexer;
    private final HiveTracker hiveTracker;
    private final ShootingPlanner planner;

    private boolean feeding;
    private final Readiness readiness = new Readiness();
    private long readySinceNanos;
    private double targetBearingDegrees = Double.NaN;
    private double distanceInches = Double.NaN;
    private ShootingPlanner.Shot shot;
    private String state = "IDLE";

    private static final class Readiness {
        boolean turretReachable;
        boolean turretSettled;
        boolean hoodSettled;
        boolean flywheel;
        boolean robotMotion;
        boolean vision;
        boolean hive;
        boolean dwell;

        boolean allBeforeDwell() {
            return turretReachable && turretSettled && hoodSettled && flywheel
                    && robotMotion && vision && hive;
        }

        void clear() {
            turretReachable = false;
            turretSettled = false;
            hoodSettled = false;
            flywheel = false;
            robotMotion = false;
            vision = false;
            hive = false;
            dwell = false;
        }
    }

    public ShootingController(
            Drive drive,
            Intake intake,
            Turret turret,
            Hood hood,
            Flywheel flywheel,
            Indexer indexer,
            HiveTracker hiveTracker
    ) {
        this.drive = drive;
        this.intake = intake;
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.indexer = indexer;
        this.hiveTracker = hiveTracker;
        planner = new ShootingPlanner();
    }

    public Command shoot() {
        return Command.build()
                .setStart(this::begin)
                .setExecute(this::execute)
                .setEnd(end -> finish())
                .requiring(intake, turret, hood, flywheel, indexer);
    }

    private void begin() {
        feeding = false;
        readySinceNanos = 0L;
        indexer.setPosition(INDEXER_RETRACTED);
        intake.setPower(INTAKE_IDLE);
        state = "ACQUIRING HIVE";
    }

    private void execute() {
        HiveTracker.Target target = hiveTracker.target(ALLIANCE);
        if (target == null) {
            shot = null;
            targetBearingDegrees = Double.NaN;
            distanceInches = Double.NaN;
            stowAndStopFeed();
            state = "WAITING FOR HIVE CELL";
            return;
        }

        distanceInches = Aiming.distanceFromTurret(drive.getPose(), target.x, target.y);
        targetBearingDegrees = Aiming.bearingFromTurret(drive.getPose(), target.x, target.y);
        readiness.turretReachable = turret.setAngle(targetBearingDegrees);
        shot = planner.getShot(distanceInches);

        if (shot != null) {
            hood.setAngle(shot.hoodDegrees);
            flywheel.setSpeed(shot.flywheelPercent);
        } else {
            hood.setAngle(HOOD_STOW_DEGREES);
            flywheel.setSpeed(0.0);
        }

        readiness.turretSettled = readiness.turretReachable && turret.isSettled();
        readiness.hoodSettled = shot != null && hood.isSettled();
        readiness.flywheel = shot != null && flywheel.atSpeed();
        readiness.robotMotion = drive.getTranslationalSpeed() <= MAX_TRANSLATIONAL_SPEED_IN_S
                && drive.getAngularSpeedDegrees() <= MAX_ANGULAR_SPEED_DEG_S;
        readiness.vision = hiveTracker.hasFreshVision();
        readiness.hive = hiveTracker.inFeedWindow();

        boolean ready = shot != null && readiness.allBeforeDwell();
        updateDwell(ready);

        feeding = ready && readiness.dwell;
        if (feeding) {
            indexer.setPosition(INDEXER_DEPLOYED);
            intake.setPower(INTAKE);
            state = hiveTracker.isEarlyFeedWindow() ? "FEEDING EARLY" : "FEEDING";
        } else {
            indexer.setPosition(INDEXER_RETRACTED);
            intake.setPower(INTAKE_IDLE);
            state = ready ? "READY DWELL" : firstBlockingReason();
        }
    }

    private void updateDwell(boolean rawReady) {
        if (!rawReady) {
            readySinceNanos = 0L;
            readiness.dwell = false;
            return;
        }
        long now = System.nanoTime();
        if (readySinceNanos == 0L) readySinceNanos = now;
        readiness.dwell = (now - readySinceNanos) / 1_000_000_000.0
                >= READY_DWELL_SECONDS;
    }

    private String firstBlockingReason() {
        if (shot == null) return "SHOT MAP NOT READY";
        if (!readiness.turretReachable) return "TURRET UNREACHABLE";
        if (!readiness.turretSettled) return "TURRET SETTLING";
        if (!readiness.hoodSettled) return "HOOD SETTLING";
        if (!readiness.flywheel) return "FLYWHEEL SPINNING";
        if (!readiness.robotMotion) return "ROBOT MOVING";
        if (!readiness.vision) return "VISION NOT FRESH";
        if (!readiness.hive) return "WAITING FOR HIVE";
        return "PREPARING";
    }

    private void stowAndStopFeed() {
        feeding = false;
        readySinceNanos = 0L;
        readiness.clear();
        indexer.setPosition(INDEXER_RETRACTED);
        intake.setPower(INTAKE_IDLE);
        hood.setAngle(HOOD_STOW_DEGREES);
        flywheel.setSpeed(0.0);
    }

    private void finish() {
        stowAndStopFeed();
        // Deliberately do not command the turret: it holds its last target.
        state = "IDLE";
    }

    public void stop() {
        finish();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shoot State", state);
        telemetry.addData("Shot Map", planner.status());
        telemetry.addData("Shoot Feeding", feeding);
        telemetry.addData("HIVE Tracker", hiveTracker.status());
        telemetry.addData("HIVE State", hiveTracker.state());
        telemetry.addData("HIVE Selected CELL", valueOrDash(hiveTracker.selectedCell()));
        telemetry.addData("HIVE Tags Used", hiveTracker.tagsUsed());
        telemetry.addData("HIVE Vision Age ms", finite(hiveTracker.visionAgeMs(), "%.0f"));
        telemetry.addData("HIVE Angle deg", finite(hiveTracker.angleDegrees(), "%.2f"));
        telemetry.addData("HIVE Rate deg/s", finite(hiveTracker.rateDegreesPerSecond(), "%.2f"));
        telemetry.addData("HIVE Time To Stable s", finite(hiveTracker.timeToStableSeconds(), "%.3f"));
        telemetry.addData("Shot Distance in", finite(distanceInches, "%.2f"));
        telemetry.addData("Shot Distance Clamped", shot != null && shot.clamped);
        telemetry.addData("Shot Hood Target deg",
                shot != null ? String.format(Locale.US, "%.2f", shot.hoodDegrees) : "--");
        telemetry.addData("Shot Flywheel Target %",
                shot != null ? String.format(Locale.US, "%.1f", shot.flywheelPercent) : "--");
        telemetry.addData("Turret Bearing deg", finite(targetBearingDegrees, "%.2f"));
        telemetry.addData("Ready/Map", shot != null);
        telemetry.addData("Ready/Turret Reachable", readiness.turretReachable);
        telemetry.addData("Ready/Turret Settled", readiness.turretSettled);
        telemetry.addData("Ready/Hood Settled", readiness.hoodSettled);
        telemetry.addData("Ready/Flywheel", readiness.flywheel);
        telemetry.addData("Ready/Robot Motion", readiness.robotMotion);
        telemetry.addData("Ready/Vision Fresh", readiness.vision);
        telemetry.addData("Ready/HIVE Window", readiness.hive);
        telemetry.addData("Ready/Dwell", readiness.dwell);
    }

    private static String finite(double value, String format) {
        return Double.isFinite(value) ? String.format(Locale.US, format, value) : "--";
    }

    private static String valueOrDash(Object value) {
        return value == null ? "--" : value.toString();
    }
}
