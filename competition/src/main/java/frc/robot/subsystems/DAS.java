package frc.robot.subsystems;

import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private NavigableMap<Double, MotorSettings> passingDistanceMap;

    private double velocityOffset = 0;

    public DAS() {
        NTHelper.setDouble("/tuning/DASOffset", DEFAULT_DISTANCE_OFFSET);
        NTHelper.setDouble("/tuning/velocityOffset", velocityOffset);
        distanceMap = new TreeMap<>();
        passingDistanceMap = new TreeMap<>();
        initializeMap();
    }

    private void initializeMap() {
       distanceMap.put(1.5, new MotorSettings(10, 2400));
        distanceMap.put(2.0, new MotorSettings(20, 2500));
        distanceMap.put(2.4, new MotorSettings(20, 2650));
        distanceMap.put(2.7, new MotorSettings(20, 2750));
        // distanceMap.put(2.9, new MotorSettings(30, 2700));
        distanceMap.put(3.2, new MotorSettings(30, 2825));
        distanceMap.put(3.7, new MotorSettings(30, 3000));
        distanceMap.put(4.0, new MotorSettings(30, 3125));
        distanceMap.put(4.3, new MotorSettings(30,3200));


        passingDistanceMap.put(5.5, new MotorSettings(55,3300));
        passingDistanceMap.put(6.7, new MotorSettings(55,4000));
        passingDistanceMap.put(8.0, new MotorSettings(55,5000));
    }

     public MotorSettings calculatePassingAS(double distance) {
        return calculateAS(distance, passingDistanceMap, 0, 0);
    }

    public MotorSettings calculateAS(double distance) {
        double distanceOffset = NTHelper.getDouble("/tuning/DASOffset", DEFAULT_DISTANCE_OFFSET);
        double velocityOffset = NTHelper.getDouble("/tuning/velocityOffset", 0);
        return calculateAS(distance, distanceMap, distanceOffset, velocityOffset);
    }

    public MotorSettings calculateAS(double distance, NavigableMap<Double, MotorSettings> distanceMap, double distanceOffset, double velocityOffset) {
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

        if (distance > 2.5) {
            interpolatedVoltage += velocityOffset;
        }

        return new MotorSettings(interpolatedAngle, interpolatedVoltage);
    }

    public Command increaseVelocityOffset() {
        return Commands.runOnce(() -> {
            velocityOffset += 50;
            NTHelper.setDouble("/tuning/velocityOffset", velocityOffset);
        });
    }

    public Command decreaseVelocityOffset() {
        return Commands.runOnce(() -> {
            velocityOffset -= 50;
            NTHelper.setDouble("/tuning/velocityOffset", velocityOffset);
        });
    }
}