package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.subsystems.DAS;

public class PassingCommands extends SubsystemBase {

    private final RobotContainer robot;

    public PassingCommands(RobotContainer robot) {
        this.robot = robot;
    }

    public Command trenchPass() {
        Pose2d[] trenchPosesBlue = { new Pose2d(6, 0.8, Rotation2d.fromDegrees(180)),
                new Pose2d(6, 7.3, Rotation2d.fromDegrees(180)) };
        Pose2d[] trenchPosesRed = { new Pose2d(10.5, 0.8, Rotation2d.fromDegrees(0)),
                new Pose2d(10.5, 7.3, Rotation2d.fromDegrees(0)) };

        Command driveToNearestTrench = Commands.either(robot.bline.goToNearestPose(trenchPosesRed),
                robot.bline.goToNearestPose(trenchPosesBlue),
                () -> PoseTransformUtils.isRedAlliance());

        Command passFuel = Commands.sequence(
                robot.intakeCommands.armDown(),
                robot.intake.outtake());

        return Commands.sequence(driveToNearestTrench, passFuel);
    }

    public Command shootPass() {
        Pose2d[] trenchPosesBlue = { new Pose2d(6, 1.75, Rotation2d.fromDegrees(180)),
                new Pose2d(6, 6.25, Rotation2d.fromDegrees(180)) };
        Pose2d[] trenchPosesRed = { new Pose2d(10.5, 1.75, Rotation2d.fromDegrees(0)),
                new Pose2d(10.5, 6.25, Rotation2d.fromDegrees(0)) };

        Command goToNearestPassingSpot = Commands.either(robot.bline.goToNearestPose(trenchPosesRed),
                robot.bline.goToNearestPose(trenchPosesBlue),
                () -> PoseTransformUtils.isRedAlliance());

        Command passFuel = Commands.parallel(
                robot.shooterCommands.revSpeedFromDAS(),
                Commands.sequence(
                        Commands.waitSeconds(3),
                        robot.shooterCommands.feed()));

        return Commands.sequence(goToNearestPassingSpot, passFuel);
    }

    private Pose2d getAimToPassPose() {
        Pose2d targetPose = Pose2d.kZero;
        if (robot.drivetrain.getPose().getY() > FieldConstants.LinesHorizontal.center) {
            if (PoseTransformUtils.isRedAlliance()) {
                targetPose = new Pose2d(FieldConstants.fieldLength,
                        6.5 + 1.5 * robot.driverController.getRightX(),
                        Rotation2d.fromDegrees(180));
            } else {
                targetPose = new Pose2d(0, 6.5 - 1.5 * robot.driverController.getRightX(),
                        Rotation2d.fromDegrees(180));
            }
        } else {
            if (PoseTransformUtils.isRedAlliance()) {
                targetPose = new Pose2d(FieldConstants.fieldLength,
                        1.5 + 1.5 * robot.driverController.getRightX(),
                        Rotation2d.fromDegrees(180));
            } else {
                targetPose = new Pose2d(0, 1.5 - 1.5 * robot.driverController.getRightX(),
                        Rotation2d.fromDegrees(180));
            }
        }
        return targetPose;
    }

    private double getDistanceToAimToPassPose() {
        return robot.shooterCommands.getDistanceBetweenPoses(robot.drivetrain.getPose(), getAimToPassPose());
    }

    public Command feedForPassing() {
        Command feed = Commands.parallel(
                robot.hood.setAngle(() -> {
                    return Degrees.of(45);
                }),
                robot.shooterCommands.feedOnly());

        return Commands.waitUntil(() -> {
            if (Robot.isSimulation()) {
                return true;
            }
            return robot.shooterLeft.isAtSetpoint();
        }).andThen(feed);

    }

    public Command aimToPass() {
        return Commands.parallel(
                robot.shooterCommands.lookAtPose(() -> {
                    return getAimToPassPose();
                }),
                robot.shooterCommands.rev(() -> {
                    double distance = getDistanceToAimToPassPose();
                    return Math.min(4500, 2000 + distance * 175);
                }));
    }

}