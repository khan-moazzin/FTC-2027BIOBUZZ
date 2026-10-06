package org.firstinspires.ftc.teamcode.planners;

import java.util.NavigableMap;
import java.util.TreeMap;

/** Shooter hardware values, empirical shot maps, and shot-readiness tolerances. */
public final class ShooterConstants {

    private ShooterConstants() {}

    // Axon MK2 servos on a REV Hub use 500-2500 us for their full programmed range.
    public static final double AXON_PWM_MIN_US = 500.0;
    public static final double AXON_PWM_MAX_US = 2500.0;
    public static final double SERVO_SETTLE_MARGIN_SECONDS = 0.08;

    // Turret: two opposed Axon MAX MK2 servos, direct 1:1.
    public static final double TURRET_MIN_DEGREES = -170.0;
    public static final double TURRET_MAX_DEGREES = 170.0;
    public static final double TURRET_CENTER_POSITION = 0.5;
    public static final double TURRET_DEGREES_PER_SERVO_UNIT = 360.0;
    public static final double TURRET_SECONDS_PER_60_DEGREES = 0.140;
    // PLACEHOLDERS: turret axis from robot center, robot frame (+forward, +left).
    public static final double TURRET_FORWARD_IN = 0.0;
    public static final double TURRET_LEFT_IN = 0.0;

    // Hood: 30T Axon MINI MK2 gear driving the 173T hood gear.
    // Zero degrees is the lowest hood position; positive angles raise it.
    public static final double HOOD_MIN_POSITION = 0.15; // PLACEHOLDER
    public static final double HOOD_MAX_POSITION = 0.85; // PLACEHOLDER
    public static final double HOOD_STOW_DEGREES = 0.0;
    public static final double HOOD_SERVO_GEAR_TEETH = 30.0;
    public static final double HOOD_GEAR_TEETH = 173.0;
    public static final double HOOD_MAX_DEGREES =
            (HOOD_MAX_POSITION - HOOD_MIN_POSITION)
                    * 360.0 * HOOD_SERVO_GEAR_TEETH / HOOD_GEAR_TEETH;
    public static final double HOOD_SECONDS_PER_60_DEGREES = 0.110;

    // Placeholders
    public static final double INDEXER_RETRACTED = 0.15;
    public static final double INDEXER_DEPLOYED = 0.85;

    // One direct-drive goBILDA 5000 Series motor.
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    public static final double FLYWHEEL_MAX_RPM = 5800.0;
    public static final double FLYWHEEL_MAX_PERCENT = 100.0;
    public static final double FLYWHEEL_TOLERANCE_PERCENT = 2.0;

    // These are two independent trial-derived maps. Their distance keys do not have to match.
    public static final NavigableMap<Double, Double> HOOD_ANGLE_MAP = new TreeMap<>();
    public static final NavigableMap<Double, Double> FLYWHEEL_SPEED_MAP = new TreeMap<>();

    static {
        // distance inches -> hood degrees
        HOOD_ANGLE_MAP.put(48.0, 10.61);

        // distance inches -> flywheel percent (0-100)
        FLYWHEEL_SPEED_MAP.put(48.0, 62.1);
    }


    // Feed gate.
    public static final double READY_DWELL_SECONDS = 0.10;
    public static final double MAX_TRANSLATIONAL_SPEED_IN_S = 6.0; // TUNER
    public static final double MAX_ANGULAR_SPEED_DEG_S = 15.0;     // TUNER

    // Alliance HIVE field positions in Pedro coordinates.
    public static final double RED_HIVE_X = 59.25;
    public static final double RED_HIVE_Y = 72.0;

    public static final double BLUE_HIVE_X = 84.75;
    public static final double BLUE_HIVE_Y = 72.0;

    public static final double CELL_TARGET_RADIUS_IN = 42.91 / 2.0 - 9.938 / 2.0;

    // Moving-HIVE vision and early-feed gate.
    public static final int LIMELIGHT_PIPELINE = 1;
    public static final double LIMELIGHT_PITCH_DEGREES = 0.0; // verify after mounting
    public static final double LIMELIGHT_ROLL_DEGREES = 0.0;  // verify after mounting
    public static final double HIVE_STABLE_ANGLE_DEGREES = 30.0;

    public static final double HIVE_STABLE_TOLERANCE_DEGREES = 5.0;
    public static final double HIVE_ANGLE_FILTER = 0.35;
    public static final double HIVE_RATE_FILTER = 0.30;
    public static final double HIVE_MIN_RATE_DEG_S = 5.0;
    public static final double HIVE_MAX_NORMAL_X = 0.45;
    public static final double HIVE_MAX_VISION_AGE_MS = 250.0;
    public static final double HIVE_AIM_MEMORY_SECONDS = 20.0;
    public static final double HIVE_EARLY_FEED_LEAD_SECONDS = 0.50; // TUNER
}
