package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FloppingUtil;

public class CenterTwiceBumpAuto extends AutoRoutine {
    public CenterTwiceBumpAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path trenchToCenterPath = AutoPaths.getPath("Trench-to-Center");
        Path centerToBumpPath = AutoPaths.getPath("Center-to-Bump");
        Path bumpToLeftoversPath = AutoPaths.getPath("Bump-to-Leftovers");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.sequence(
                Commands.deadline(
                        trenchToCenter,
                        Commands.sequence(
                                robot.intakeCommands.armDown(),
                                Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                                robot.intake.intake()))
                        .withTimeout(5),
                robot.intake.intake());

        Command bumpToLeftoversAndIntake = Commands.deadline(
                bumpToLeftovers,
                Commands.sequence(
                        robot.intakeCommands.armDown(),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 6),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 12),
                        robot.shooterCommands.rev(() -> 3000.0)));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        robot.shooterCommands.rev(() -> 3000.0)),
                // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >=
                // 4)
                // .andThen(robot.intake.stop())),
                //robot.drivetrain.idle(),
                robot.shooterCommands.scoreDeadline(4.5),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }
}
