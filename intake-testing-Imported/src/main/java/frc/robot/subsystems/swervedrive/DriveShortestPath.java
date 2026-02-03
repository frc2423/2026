package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveShortestPath {
    private final SwerveSubsystem swerve;
    private final BLine bline;

    // points including bumps
    // Translation2d close[] = {new Translation2d(3.5,0.5), new
    // Translation2d(3.5,2.4), new Translation2d(3.5,5.6), new
    // Translation2d(3.5,7.5)};
    // Translation2d closeNeutral[] = {new Translation2d(6,0.5), new
    // Translation2d(6,2.4), new Translation2d(6,5.6), new Translation2d(6,7.5)};
    // Translation2d farNeutral[] = {new Translation2d(10.5,0.5), new
    // Translation2d(10.5,2.4), new Translation2d(10.5,5.6), new
    // Translation2d(10.5,7.5)};
    // Translation2d far[] = {new Translation2d(13,0.5), new Translation2d(13,2.4),
    // new Translation2d(13,5.6), new Translation2d(13,7.5)};

    // just trenches
    private static final Translation2d close[] = { new Translation2d(3.5, 0.5), new Translation2d(3.5, 7.5) };
    private static final Translation2d closeNeutral[] = { new Translation2d(6, 0.5), new Translation2d(6, 7.5) };
    private static final Translation2d farNeutral[] = { new Translation2d(10.5, 0.5), new Translation2d(10.5, 7.5) };
    private static final Translation2d far[] = { new Translation2d(13, 0.5), new Translation2d(13, 7.5) };
    private static final Translation2d groups[][] = { close, closeNeutral, farNeutral, far };

    enum Direction {
        CLOSE,
        FAR,
    }

    public DriveShortestPath(SwerveSubsystem swerve, BLine bline) {
        this.swerve = swerve;
        this.bline = bline;
    }

    private Pose2d getPose() {
        return swerve.getPose();
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
        int closestIndex = 0;
        for (int i = 1; i < groups.length; i++) { // check if any other group is closer
            Direction groupDirection = getDirection(getPose().getTranslation(), groups[i][0]);
            double distanceToClosestPose = getPose().getX() - groups[closestIndex][0].getX();
            double distanceToPose = getPose().getX() - groups[i][0].getX();
            boolean isSameDirection = pathDirection == groupDirection;
            if (Math.abs(distanceToPose) < Math.abs(distanceToClosestPose)
                    && isSameDirection) {
                closestIndex = i;
            }
        }
        return closestIndex;
    }

  /**
   * 
   * @param targetPose2d
   *  assuming not currently on bump or in trench
   * @return
   */
  public Command driveShortestPath(Pose2d targetPose2d) {

      Direction pathDirection = getDirection(getPose().getTranslation(), targetPose2d.getTranslation());
      int closestGroupIndex = findIndexOfClosestGroup(groups, pathDirection);

      if (pathDirection == Direction.FAR) {
        Translation2d previousPoint = getPose().getTranslation();
        Translation2d nextPoint = findClosestPoint(groups[closestGroupIndex], getPose().getTranslation());
        for(int i = closestGroupIndex + 1; nextPoint.getX() < targetPose2d.getX(); i++) {
          previousPoint = nextPoint;
          nextPoint = findClosestPoint(groups[i], previousPoint);
          // add point to bline
        }
      } else {
        Translation2d previousPoint = getPose().getTranslation();
        Translation2d nextPoint = findClosestPoint(groups[closestGroupIndex], getPose().getTranslation());
        for(int i = closestGroupIndex - 1; nextPoint.getX() > targetPose2d.getX(); i--) {
          previousPoint = nextPoint;
          nextPoint = findClosestPoint(groups[i], previousPoint);
          // add point to bline
        }
      }
      return Commands.none();
  }

}
