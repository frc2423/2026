package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.lib.BLine.*;

public class BLine {

    CommandSwerveDrivetrain swerve;
    FollowPath.Builder pathBuilder;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public BLine(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;

        Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                4.4, 12.0,
                540, 860,
                0.03, 2.0,
                .2));
        pathBuilder = new FollowPath.Builder(
                swerve,
                () -> swerve.getState().Pose,
                () -> swerve.getState().Speeds,
                (chassisSpeeds) -> {
                    SwerveRequest.RobotCentric swerveRequest = drive
                            .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                            .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                            .withRotationalRate(
                                    chassisSpeeds.omegaRadiansPerSecond);
                    swerve.setControl(swerveRequest);
                },
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0)); // .withDefaultShouldFlip();

        FollowPath.setDoubleLoggingConsumer(pair -> {
            NTHelper.setDouble("/bline/double/" + pair.getFirst(), pair.getSecond());
        });

        FollowPath.setBooleanLoggingConsumer(pair -> {
            NTHelper.setBoolean("/bline/boolean/" + pair.getFirst(), pair.getSecond());
        });

        FollowPath.setPoseLoggingConsumer(pair -> {
            NTHelper.setPose("/bline/pose/" + pair.getFirst(), pair.getSecond());
        });

        FollowPath.setTranslationListLoggingConsumer(pair -> {
            Translation2d[] blineTranslations = pair.getSecond();
            Translation2d[] translations = new Translation2d[pair.getSecond().length + 1];
            translations[0] = swerve.getState().Pose.getTranslation();
            for (int i = 0; i < blineTranslations.length; i++) {
                translations[i + 1] = blineTranslations[i];
            }
            NTHelper.setTranslationArray("/bline/translationList/" + pair.getFirst(), translations);
        });
    }

    public Command goToPose(Pose2d pose) {
        Path testPoseProfeciency = new Path(new Path.Waypoint(pose));
        return pathBuilder.build(testPoseProfeciency);
    }

    public Command goToNearestPose(Pose2d[] targetPoses) {

        return Commands.defer(() -> {
            double shortestDistance = 1000;
            Pose2d poseChosen = targetPoses[0];

            for (Pose2d pose : targetPoses) {
                double distance = swerve.getPose().getTranslation().getDistance(pose.getTranslation());
                if (distance < shortestDistance) {
                    shortestDistance = distance;
                    poseChosen = pose;
                }

            }

            return goToPose(poseChosen);
        }, Set.of(swerve));

    }
}
