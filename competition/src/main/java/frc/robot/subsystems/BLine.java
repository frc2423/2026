package frc.robot.subsystems;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

// import java.nio.file.Path;
// package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.*;
// import swervelib.SwerveDrive;

public class BLine {

        CommandSwerveDrivetrain swerve;
        FollowPath.Builder pathBuilder;

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        public BLine(CommandSwerveDrivetrain swerve) {
                this.swerve = swerve;

                Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                                4.5, 12.0,
                                540, 860,
                                0.03, 2.0,
                                .2));
                pathBuilder = new FollowPath.Builder(
                                swerve,
                                () -> swerve.getState().Pose,
                                () -> swerve.getState().Speeds,
                                (chassisSpeeds) -> {
                                        FieldCentric swerveRequest = drive
                                                        .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                                                        .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                                                        .withRotationalRate(
                                                                        chassisSpeeds.omegaRadiansPerSecond);
                                        swerve.setControl(swerveRequest);
                                },
                                // chassisSpeeds->swerve.getKinematics().toSwerveModuleStates(chassisSpeeds),
                                new PIDController(5.0, 0.0, 0.0),
                                new PIDController(3.0, 0.0, 0.0),
                                new PIDController(0.0, 0.0, 0.0)); // .withDefaultShouldFlip();
                // .withPoseReset(swerve::resetOdometry);

                log();
        }

        public void log() {
                FollowPath.setDoubleLoggingConsumer(pair -> {
                        // Logger.recordOutput(pair.getFirst(), pair.getSecond());
                        NTHelper.setDouble("/bline/" + pair.getFirst(), pair.getSecond());
                });

                // FollowPath.setBooleanLoggingConsumer(pair -> {
                // Logger.recordOutput(pair.getFirst(), pair.getSecond());
                // });

                // FollowPath.setPoseLoggingConsumer(pair -> {
                // Logger.recordOutput(pair.getFirst(), pair.getSecond());
                // });

                // FollowPath.setTranslationListLoggingConsumer(pair -> {
                // Logger.recordOutput(pair.getFirst(), pair.getSecond());
                // });
        }

        private Command followCommand1() {

                Path startingGoFoward = new Path(
                                // new Path.Waypoint(new Translation2d(0, 0), new Rotation2d(0)),
                                // new Path.Waypoint(new Translation2d(1.0, 4), new Rotation2d(0)),
                                // new Path.Waypoint(new Translation2d(3, 0.5), new Rotation2d(0)),
                                new Path.Waypoint(new Translation2d(2, 1), new Rotation2d(Math.PI)));

                // startingGoFoward.setElement(0,
                // new Path.Waypoint(new Translation2d(swerve.getPose().getX(),
                // swerve.getPose().getY()),
                // new Rotation2d(0)));

                Command followCommand = pathBuilder.build(startingGoFoward);

                return followCommand;
        }

        public Command goToPose(Pose2d pose) {
                Path testPoseProfeciency = new Path(
                                // new Path.Waypoint(new Translation2d(1,4), new Rotation2d(0))
                                new Path.Waypoint(pose.getTranslation(), pose.getRotation()));

                Command goToTargetPose = pathBuilder.build(testPoseProfeciency);
                return goToTargetPose;
        }

        private Command followCommand2() {
                Path startingGoFoward = new Path(
                                new Path.Waypoint(new Translation2d(3, 1), new Rotation2d(Math.PI)));

                Command followCommand2 = pathBuilder.build(startingGoFoward);

                return followCommand2;
        }

        private Command followCommand3() {
                Path startingGoFoward = new Path(
                                new Path.Waypoint(new Translation2d(1.0, 0.0), new Rotation2d(0)),
                                new Path.Waypoint(new Translation2d(5.0, 5.0), new Rotation2d(-30)));

                Command followCommand2 = pathBuilder.build(startingGoFoward);

                return followCommand2;
        }

}
