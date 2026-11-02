package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class OutpostAndDepotAuto extends AutoRoutine {
    public OutpostAndDepotAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path trenchToOutpostPath = AutoPaths.getPath("Trench-to-Outpost");
        Path outpostToDepotPath = AutoPaths.getPath("Outpost-to-Depot");

        if (PoseTransformUtils.isRedAlliance()) {
            trenchToOutpostPath.flip();
            outpostToDepotPath.flip();
        }

        FollowPath trenchToOutpost = robot.bline.pathBuilder.build(trenchToOutpostPath);
        FollowPath outpostToDepot = robot.bline.pathBuilder.build(outpostToDepotPath);

        return Commands.sequence(
                trenchToOutpost,
                Commands.waitSeconds(2),
                Commands.deadline(
                        outpostToDepot,
                        Commands.waitUntil(() -> outpostToDepot.getCurrentTranslationElementIndex() >= 4)
                                .andThen(startIntaking())),
                robot.intake.stop(),
                goToHubAndShoot());
    }
}
