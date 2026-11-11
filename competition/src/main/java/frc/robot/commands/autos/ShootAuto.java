package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootAuto extends AutoRoutine {
    public ShootAuto(RobotContainer robot) {
        super(robot);
    }

    @Override
    public Command build() {
        return goToHubAndShoot();

        // Commands.sequence(shooter.prepareToShoot(),
        // shooter.spinFeeder(() -> feederSpeed));
    }
}
