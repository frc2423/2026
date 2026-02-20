package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.*;
import frc.robot.lib.BLine.Path.PathConstraints;

public class DriveShortestPath {
    private final CommandSwerveDrivetrain swerve;
    private final BLine bline;

    private final static double closeX = 3.0;
    private final static double closeNeutralX = 6.5;
    private final static double farNeutralX = 10;
    private final static double farX = 14;
    private final static double groupXs[] = { closeX, closeNeutralX, farNeutralX, farX };

    private final static double leftTrenchY = 7.25;
    private final static double leftBumpY = 5.6;
    private final static double rightBumpY = 3.5;
    private final static double rightTrenchY = .75;
 
    private final static double trenchOnlyYs[] = { leftTrenchY, rightTrenchY };
    private final static double ys[] = { leftTrenchY, leftBumpY, rightBumpY, rightTrenchY };

    private static final Translation2d groupsTrenchAndBumps[][] = generateGroup(groupXs, ys);
    private static final Translation2d groupsTrenchOnly[][] = generateGroup(groupXs, trenchOnlyYs);

    enum Direction {
        CLOSE,
        FAR,
    }

    public DriveShortestPath(CommandSwerveDrivetrain swerve, BLine bline) {
        this.swerve = swerve;
        this.bline = bline;
    }

    private Pose2d getPose() {
        return swerve.getPose();
    }

    private static final Translation2d[][] generateGroup(double[] xs, double[] ys) {
        Translation2d groups[][] = new Translation2d[xs.length][ys.length];

        for (int xIndex = 0; xIndex < xs.length; xIndex++) {
            Translation2d group[] = new Translation2d[ys.length];
            for (int yIndex = 0; yIndex < ys.length; yIndex++) {
                group[yIndex] = new Translation2d(xs[xIndex], ys[yIndex]);
            }
            groups[xIndex] = group;
        }
        return groups;
    }

    /**
     * finds the next point in line based on an inputed point
     * 
     * @param points
     * @return
     */
    private Translation2d findClosestPoint(Translation2d[] points, Translation2d currentPoint) {
        Translation2d nextPoint = points[0];
        for (int i = 1; i < points.length; i++) { // check if any other point is shorter
            if (getPose().getTranslation().getDistance(points[i]) < getPose().getTranslation().getDistance(nextPoint)) {
                nextPoint = points[i];
            }
        }
        return nextPoint;
    }

    private static Direction getDirection(Translation2d currentPose, Translation2d targetPose) {
        return currentPose.getX() < targetPose.getX() ? Direction.FAR : Direction.CLOSE;
    }

    /**
     * finds the next point in line based on an inputed point
     * 
     * @param points
     * @return
     */
    private int findIndexOfClosestGroup(Translation2d groups[][], Direction pathDirection) {
        // int closestIndex = 0;
        // Direction groupDirection = getDirection(getPose().getTranslation(),
        // groups[closestIndex][0]);
        // for(closestIndex = 0; groupDirection != Direction.FAR; closestIndex++) {
        // groupDirection = getDirection(getPose().getTranslation(),
        // groups[closestIndex][0]);
        // }
        int closestIndex = -1;

        for (int i = 0; i < groups.length; i++) { // check if any other group is closer
            Direction groupDirection = getDirection(getPose().getTranslation(), groups[i][0]);
            double distanceToClosestPose = closestIndex == -1 ? 10000000000.0
                    : getPose().getX() - groups[closestIndex][0].getX();
            double distanceToPose = getPose().getX() - groups[i][0].getX();
            boolean isSameDirection = pathDirection == groupDirection;
            if (Math.abs(distanceToPose) < Math.abs(distanceToClosestPose)
                    && isSameDirection) {
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    public Rotation2d getRotation(Direction pathDirection) {
        Rotation2d rotation;
        if (pathDirection == Direction.CLOSE) {
            rotation = new Rotation2d(Math.PI);
        } else {
            rotation = new Rotation2d(0);
        }
        return rotation;
    }

    public Rotation2d getClosestRotation() {
        Rotation2d closestRotation2d = new Rotation2d(0);
        if (getPose().getRotation().getDegrees() > 90 & getPose().getRotation().getDegrees() < 270) {
            closestRotation2d = new Rotation2d(Math.PI);
        }
        return closestRotation2d;
    }

    public Command driveShortestPath(Pose2d targetPose2d) {
        return driveShortestPath(targetPose2d, false, false);
    }

    /**
     * 
     * @param targetPose2d
     *                     assuming not currently on bump or in trench
     * @return
     */
    public Command driveShortestPath(Pose2d inputTargetPose2d, boolean goesOverBumps, boolean flipForRedAlliance) {
        Translation2d groups[][] = goesOverBumps ? groupsTrenchAndBumps : groupsTrenchOnly;
        Pose2d targetPose2d = (PoseTransformUtils.isRedAlliance()) ? FlippingUtil.flipFieldPose(inputTargetPose2d) : inputTargetPose2d;
        Supplier<Command> someCommand = () -> {
            
            
            
            Direction pathDirection = getDirection(getPose().getTranslation(), targetPose2d.getTranslation());
            int closestGroupIndex = findIndexOfClosestGroup(groups, pathDirection);
            double handoffRadius = 0.2;

            List<Path.PathElement> waypoints = new ArrayList<>();
            // waypoints.add(new Path.RotationTarget(Rotation2d.k180deg, .5, false));

            if (closestGroupIndex < 0) {
                // don't do anything
            } else if (pathDirection == Direction.FAR) {
                Translation2d previousPoint = getPose().getTranslation();
                Translation2d nextPoint = findClosestPoint(groups[closestGroupIndex], getPose().getTranslation());
                Rotation2d rotationTarget = getClosestRotation();
                int i;
                for (i = closestGroupIndex + 1; nextPoint.getX() < targetPose2d.getX() && i < groups.length; i++) {
                    // if (getPose().getRotation().getDegrees() > 0 &&
                    // getPose().getRotation().getDegrees() < 180) {
                    // rotationTarget = new Rotation2d(45);
                    // } else {
                    // rotationTarget = new Rotation2d(315);
                    // }
                    waypoints.add(new Path.Waypoint(nextPoint, handoffRadius, rotationTarget, false));
                    previousPoint = nextPoint;
                    nextPoint = findClosestPoint(groups[i], previousPoint);
                }
                if (i >= groups.length) {
                    waypoints.add(new Path.Waypoint(nextPoint, handoffRadius, getClosestRotation(), false));
                }

            } else if (closestGroupIndex >= 0) {
                Translation2d previousPoint = getPose().getTranslation();
                Translation2d nextPoint = findClosestPoint(groups[closestGroupIndex], getPose().getTranslation());
                Rotation2d rotationTarget = getClosestRotation();
                int i;
                for (i = closestGroupIndex - 1; nextPoint.getX() > targetPose2d.getX() && i >= 0; i--) {
                    // if (getPose().getRotation().getDegrees() > 0 &&
                    // getPose().getRotation().getDegrees() < 180) {
                    // rotationTarget = new Rotation2d(135);
                    // } else {
                    // rotationTarget = new Rotation2d(225);
                    // }
                    waypoints.add(new Path.Waypoint(nextPoint, handoffRadius, rotationTarget, false));
                    previousPoint = nextPoint;
                    nextPoint = findClosestPoint(groups[i], previousPoint);
                }
                if (i < 0) {
                    waypoints.add(new Path.Waypoint(nextPoint, handoffRadius, getClosestRotation(), false));
                }
            }

            waypoints.add(new Path.Waypoint(targetPose2d));
            
            PathConstraints pathConstraints = new PathConstraints()
                .setMaxVelocityMetersPerSec(2)
                .setMaxAccelerationMetersPerSec2(12);
            FollowPath followPath = bline.pathBuilder.build(new Path(pathConstraints,waypoints.toArray(new Path.PathElement[0])));
            return followPath;
        };

        return Commands.defer(someCommand, Set.of(swerve));
    }

}
