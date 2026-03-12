package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

import frc.robot.NTHelper;

// DAS means distance angle speed table
public class DAS {
    public static final double DEFAULT_DISTANCE_OFFSET = 0;

    public class MotorSettings {
        public final double angle; // in degrees
        public final double velocity; // in volts

        public MotorSettings(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
        }

        public double getAngle() {
            return angle;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    private NavigableMap<Double, MotorSettings> distanceMap; // Map from distance to settings

    public DAS() {
        NTHelper.setDouble("/tuning/DASOffset", DEFAULT_DISTANCE_OFFSET);
        distanceMap = new TreeMap<>();
        initializeMap();
    }

    private void initializeMap() {
        // distanceMap.put(1.4, new MotorSettings(0, 2700));
        // distanceMap.put(2.0, new MotorSettings(14, 2700));
        // distanceMap.put(2.6, new MotorSettings(18.5, 2900));
        // distanceMap.put(3.2, new MotorSettings(18.5, 3100));
        // distanceMap.put(3.8, new MotorSettings(23.5, 3250));
        // distanceMap.put(4.4, new MotorSettings(27, 3450));
        // distanceMap.put(5.0, new MotorSettings(32, 3550));

        distanceMap.put(1.4, new MotorSettings(5, 2900));
        distanceMap.put(2.0, new MotorSettings(14, 2900));
        distanceMap.put(2.6, new MotorSettings(18.75, 3100));
        distanceMap.put(3.2, new MotorSettings(23, 3100));
        distanceMap.put(3.8, new MotorSettings(23.5, 3250));
        distanceMap.put(4.4, new MotorSettings(27, 3450));
        distanceMap.put(5.0, new MotorSettings(32, 3550));
    }

    public MotorSettings calculateAS(double distance) {
        double distanceOffset = NTHelper.getDouble("/tuning/DASOffset", DEFAULT_DISTANCE_OFFSET);
        distance = distance + distanceOffset;
        // Direct match

        if (distanceMap.containsKey(distance)) {
            return distanceMap.get(distance);
        }

        // Find the closest lower and higher keys for interpolation
        Double lowerKey = distanceMap.lowerKey(distance);
        Double higherKey = distanceMap.higherKey(distance);

        // Edge cases: no lower or higher keys
        if (lowerKey == null) {
            return distanceMap.get(higherKey);
        }
        if (higherKey == null) {
            return distanceMap.get(lowerKey);
        }

        // Interpolation
        MotorSettings lowerSettings = distanceMap.get(lowerKey);
        MotorSettings higherSettings = distanceMap.get(higherKey);

        // Calculate weighted average
        double ratio = (distance - lowerKey) / (higherKey - lowerKey);
        double interpolatedAngle = lowerSettings.getAngle()
                + ratio * (higherSettings.getAngle() - lowerSettings.getAngle());
        double interpolatedVoltage = lowerSettings.getVelocity()
                + ratio * (higherSettings.getVelocity() - lowerSettings.getVelocity());

        return new MotorSettings(interpolatedAngle, interpolatedVoltage);
    }
}