package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class DepotAuto extends AutoRoutine {
    public DepotAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path bumpToDepotPath = AutoPaths.getPath("Bump-to-Depot");

        if (PoseTransformUtils.isRedAlliance()) {
            bumpToDepotPath.flip();
        }

        FollowPath bumpToDepot = robot.bline.pathBuilder.build(bumpToDepotPath);

        Command bumpToDepotCommand = Commands.deadline(
            bumpToDepot,
            Commands.sequence(
                Commands.waitUntil(() -> bumpToDepot.getCurrentTranslationElementIndex() >= 1),
                robot.intakeCommands.armDown(),
                robot.intake.intake()));

        return Commands.sequence(
                bumpToDepotCommand,
                robot.intake.stop(),
                goToHubAndShoot());
    }
}
