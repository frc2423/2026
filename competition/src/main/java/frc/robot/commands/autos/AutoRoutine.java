package frc.robot.commands.autos;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.subsystems.FloppingUtil;

/**
 * Base class for autos. Each auto builds its command in {@link #build()},
 * loading
 * fresh copies of its paths so flipping/flopping never affects another auto.
 */
public abstract class AutoRoutine {
    protected static final Pose2d shootInFrontOfHubPose = new Pose2d(2.5, 4, Rotation2d.fromDegrees(-180));

    protected final RobotContainer robot;

    public AutoRoutine(RobotContainer robot) {
        this.robot = robot;
    }

    public abstract Command build();

    protected Command startIntaking() {
        return Commands.sequence(
                robot.intakeCommands.armDown(),
                robot.intake.intake());
    }

    protected Command driveToPose(Pose2d pose) {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            pose = FloppingUtil.flop(pose);
        }
        return robot.bline.goToPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
    }

    protected Command goToHubAndShoot() {
        Command driveToHub = driveToPose(shootInFrontOfHubPose);
        return Commands.sequence(
                Commands.deadline(driveToHub,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(20),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

    protected Command shoot() {
        return Commands.sequence(
                Commands.deadline(
                        robot.shooterCommands.rev(() -> 3000.0)),
                // robot.shooterCommands.scoreDeadline(3),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

    protected Command goToHubAndShootUntil(BooleanSupplier condition) {
        Command driveToHub = driveToPose(shootInFrontOfHubPose);
        return Commands.sequence(
                Commands.deadline(driveToHub,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreUntil(condition),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }
}
