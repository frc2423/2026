package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class OutpostAuto extends AutoRoutine {
    public OutpostAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path bumpToOutpostPath = AutoPaths.getPath("Bump-to-Outpost");

        if (PoseTransformUtils.isRedAlliance()) {
            bumpToOutpostPath.flip();
        }

        FollowPath bumpToOutpost = robot.bline.pathBuilder.build(bumpToOutpostPath);

        return Commands.sequence(
                bumpToOutpost,
                Commands.waitSeconds(2),
                goToHubAndShoot());
    }
}
