package org.firstinspires.ftc.teamcode;

import java.util.Map;
import java.util.NavigableMap;

import static org.firstinspires.ftc.teamcode.ShooterConstants.*;

/** Looks up the empirical hood position and flywheel RPM for a distance in inches. */
public final class ShootingPlanner {

    public static final class Shot {
        public final double hoodPosition;
        public final double flywheelRpm;
        public final boolean clamped;

        private Shot(double hoodPosition, double flywheelRpm, boolean clamped) {
            this.hoodPosition = hoodPosition;
            this.flywheelRpm = flywheelRpm;
            this.clamped = clamped;
        }
    }

    public Shot getShot(double distanceInches) {
        if (!isReady() || !Double.isFinite(distanceInches) || distanceInches < 0.0) return null;

        return new Shot(
                interpolate(HOOD_MAP, distanceInches),
                interpolate(FLYWHEEL_MAP, distanceInches),
                outside(HOOD_MAP, distanceInches) || outside(FLYWHEEL_MAP, distanceInches));
    }

    public boolean isReady() {
        return validMap(HOOD_MAP, HOOD_MIN, HOOD_MAX)
                && validMap(FLYWHEEL_MAP, 0.0, FLYWHEEL_MAX_RPM);
    }

    public String status() {
        return isReady() ? "READY" : "ADD HOOD/RPM MAP POINTS";
    }

    private static double interpolate(NavigableMap<Double, Double> map, double distance) {
        Map.Entry<Double, Double> low = map.floorEntry(distance);
        Map.Entry<Double, Double> high = map.ceilingEntry(distance);
        if (low == null) return map.firstEntry().getValue();
        if (high == null) return map.lastEntry().getValue();
        if (low.getKey().equals(high.getKey())) return low.getValue();

        double t = (distance - low.getKey()) / (high.getKey() - low.getKey());
        return low.getValue() + t * (high.getValue() - low.getValue());
    }

    private static boolean outside(NavigableMap<Double, Double> map, double distance) {
        return distance < map.firstKey() || distance > map.lastKey();
    }

    private static boolean validMap(
            NavigableMap<Double, Double> map,
            double minimumValue,
            double maximumValue
    ) {
        if (map.isEmpty()) return false;
        for (Map.Entry<Double, Double> point : map.entrySet()) {
            if (!Double.isFinite(point.getKey()) || point.getKey() < 0.0
                    || !Double.isFinite(point.getValue())
                    || point.getValue() < minimumValue
                    || point.getValue() > maximumValue) {
                return false;
            }
        }
        return true;
    }
}
