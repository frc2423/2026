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

public class CenterOnceBumpDepotOrOutpostAuto extends AutoRoutine {
    public CenterOnceBumpDepotOrOutpostAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path trenchToCenterPath = AutoPaths.getPath("Trench-to-Center");
        Path centerToBumpPath = AutoPaths.getPath("Center-to-Bump");

        boolean outpost = true;
        Path bumpToDepotOrOutpostPath = AutoPaths.getPath("Bump-to-Outpost");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            bumpToDepotOrOutpostPath = AutoPaths.getPath("Bump-to-Depot");
            outpost = false;
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToDepotOrOutpostPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToDepotOrOutpost = robot.bline.pathBuilder.build(bumpToDepotOrOutpostPath);

        Command bumpToDepotOrOutpostAndWait = bumpToDepotOrOutpost;
        if (outpost) {
            bumpToDepotOrOutpostAndWait = Commands.sequence(bumpToDepotOrOutpost, Commands.waitSeconds(2));
        }

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 2)
                                .andThen(robot.intake.stop()),
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(outpost ? 3 : 5),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                bumpToDepotOrOutpostAndWait,
                robot.intake.stop(),
                goToHubAndShoot());
    }
}
