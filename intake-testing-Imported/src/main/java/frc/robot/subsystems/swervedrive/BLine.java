package frc.robot.subsystems.swervedrive;

import java.util.function.Supplier;

// import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.*;
import swervelib.SwerveDrive;

public class BLine {

        SwerveSubsystem swerve;
        IntakeSubsystem intake;
        FollowPath.Builder pathBuilder;

        public BLine(SwerveSubsystem swerve, IntakeSubsystem intake) {
                this.swerve = swerve;
                this.intake = intake;

                Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                                4.5, 12.0, 540, 860, 0.03, 2.0, 0.2));
                pathBuilder = new FollowPath.Builder(
                                swerve,
                                swerve::getPose,
                                swerve::getRobotVelocity,
                                swerve::drive,
                                new PIDController(5.0, 0.0, 0.0),
                                new PIDController(3.0, 0.0, 0.0),
                                new PIDController(2.0, 0.0, 0.0)).withDefaultShouldFlip();
                // .withPoseReset(swerve::resetOdometry);
        }

        public Command getAutoCommandFromName(String name) {
                switch (name) {
                        case "goFowardGoBackIntake":
                                return goFowardGoBackIntake();

                        case "fancyDoubleIntake":
                                return fancyDoubleIntake();

                        case "goToPose":
                                return goToPose();

                        default:
                                break;
                }

                return Commands.none();
        };

        private Command goFowardGoBackIntake() {
                var deadline1 = Commands.deadline(followCommand1(), intake.intake());

                var auto2 = Commands.sequence(deadline1, Commands.runOnce(() -> swerve.drive(new ChassisSpeeds())),
                                Commands.waitSeconds(2), intake.stop(), followCommand2());

                return auto2;
        };

        private Command fancyDoubleIntake() {
                var deadline1 = Commands.deadline(followCommand1(), intake.intake());

                var auto2 = Commands.sequence(deadline1, Commands.runOnce(() -> swerve.drive(new ChassisSpeeds())),
                                Commands.waitSeconds(2), followCommand2(), followCommand3(), intake.stop());

                return auto2;
        };

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

        private Command goToPose() {
                Path testPoseProfeciency = new Path(
                        new Path.Waypoint(new Translation2d(1,4), new Rotation2d(0))
                );

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
