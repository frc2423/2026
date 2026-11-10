package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoPaths;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.BLine;

public class DoubleScoreDock extends AutoRoutine {
    public DoubleScoreDock(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path DoubleScoreDock_path = AutoPaths.getPath("Double-Score-Dock");

        FollowPath followPath = robot.bline.pathBuilder.build(DoubleScoreDock_path);
        robot.drivetrain.resetPose(DoubleScoreDock_path.getStartPose());

        Command shootLane = Commands.sequence(
                shoot().withTimeout(3),
                Commands.waitSeconds(1.22),
                shoot());

        return Commands.parallel(followPath, shootLane);
    }
}
