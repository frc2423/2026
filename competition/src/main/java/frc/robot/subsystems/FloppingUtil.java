package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.Waypoint;

public class FloppingUtil {
    /*
     * *****name should be changed*****
     * flips the pose over the x axis.
     */
    public static Pose2d flop(Pose2d unflippedPose2d) {
        boolean needsToFlip = true;
        if (PoseTransformUtils.isRedAlliance()) {
            needsToFlip = !needsToFlip;
        }
        double flippedY = PoseTransformUtils.FIELD_WIDTH_METERS - unflippedPose2d.getY();
        Rotation2d flippedRotation2d = new Rotation2d((2 * Math.PI) - unflippedPose2d.getRotation().getRadians());
        Pose2d flippedPose2d = needsToFlip ? new Pose2d(unflippedPose2d.getX(), flippedY, flippedRotation2d)
                : unflippedPose2d;
        return flippedPose2d;
    }

    public static Rotation2d flop(Rotation2d unflippedRotation2d) {
        return new Rotation2d((2 * Math.PI) - unflippedRotation2d.getRadians());
    }

    public static Translation2d flop(Translation2d unflippedTranslation2d) {
        double flippedY = PoseTransformUtils.FIELD_WIDTH_METERS - unflippedTranslation2d.getY();
        return new Translation2d(unflippedTranslation2d.getX(), flippedY);
    }

    public static void flopPath(Path unflippedPath) {
        List<PathElement> pathElements = unflippedPath.getPathElements();
        for (int i = 0; i < pathElements.size(); i++) {
            Path.PathElement element = pathElements.get(i);
            if (element instanceof Path.TranslationTarget) {
                pathElements.set(i, new Path.TranslationTarget(
                        flop(((Path.TranslationTarget) element).translation()),
                        ((Path.TranslationTarget) element).intermediateHandoffRadiusMeters()));
            } else if (element instanceof Path.RotationTarget) {
                pathElements.set(i, new Path.RotationTarget(
                        flop(((Path.RotationTarget) element).rotation()),
                        ((Path.RotationTarget) element).t_ratio(),
                        ((Path.RotationTarget) element).profiledRotation()));
            } else if (element instanceof Waypoint) {
                pathElements.set(i, new Waypoint(
                        new Path.TranslationTarget(
                                flop(((Path.Waypoint) element).translationTarget().translation()),
                                ((Path.Waypoint) element).translationTarget().intermediateHandoffRadiusMeters()),
                        new Path.RotationTarget(
                                flop(((Path.Waypoint) element).rotationTarget().rotation()),
                                ((Path.Waypoint) element).rotationTarget().t_ratio(),
                                ((Path.Waypoint) element).rotationTarget().profiledRotation())));
            } else if (element instanceof Path.EventTrigger) {
                pathElements.set(i, new Path.EventTrigger(
                        ((Path.EventTrigger) element).t_ratio(),
                        ((Path.EventTrigger) element).libKey()));
            }
        }
        unflippedPath.setPathElements(pathElements);
    }
}
