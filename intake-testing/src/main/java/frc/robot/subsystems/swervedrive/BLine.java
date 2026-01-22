package frc.robot.subsystems.swervedrive;

import java.util.function.Supplier;

// import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.*;
import swervelib.SwerveDrive;

public class BLine {

        SwerveSubsystem swerve;
        FollowPath.Builder pathBuilder;

        public BLine(SwerveSubsystem swerve) {
                this.swerve = swerve;

                Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
                                4.5, 12.0, 540, 860, 0.03, 2.0, 0.2));
                pathBuilder = new FollowPath.Builder(
                                swerve,
                                swerve::getPose,
                                swerve::getRobotVelocity,
                                swerve::drive,
                                new PIDController(5.0, 0.0, 0.0),
                                new PIDController(3.0, 0.0, 0.0),
                                new PIDController(2.0, 0.0, 0.0)).withDefaultShouldFlip()
                                                .withPoseReset(swerve::resetOdometry);
        }

        public Command followCommand() {

                // Path myPath = new Path("example_a.json");
                Path myPath = new Path(
                                new Path.Waypoint(new Translation2d(1.0, 0.0), new Rotation2d(0)),
                                new Path.Waypoint(new Translation2d(3.0, 0.0), new Rotation2d(0)));

                Command followCommand = pathBuilder.build(myPath);
                return followCommand;
        }
}
