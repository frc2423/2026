package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.Path;

public class ChargedUpTestAuto extends AutoRoutine {
    public ChargedUpTestAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        Path testPath = AutoPaths.getPath("chargedup-autos", "test");

        if (PoseTransformUtils.isRedAlliance()) {
            testPath.flip();
        }

        return robot.bline.pathBuilder.build(testPath);
    }
}
