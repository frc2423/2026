package frc.robot.commands.autos;

import frc.robot.RobotContainer;

public class MechanicalAdvantageDelayAuto extends CustomAuto {
    public MechanicalAdvantageDelayAuto(RobotContainer robot) {
        super(robot, new String[] { "shoot", "bump", "bump" }, new double[] { 4.5, 14 });
    }
}
