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

public class CenterOnceBumpAuto extends AutoRoutine {
    public CenterOnceBumpAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path trenchToCenterPath = AutoPaths.getPath("Trench-to-Center");
        Path bumpToLeftoversPath = AutoPaths.getPath("Bump-to-Leftovers");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToLeftovers,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.intake.stop(),
                robot.shooterCommands.scoreDeadline(20),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }
}
