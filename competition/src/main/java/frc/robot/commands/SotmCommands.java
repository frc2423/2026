package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DAS;
import frc.robot.utils.sotm.ShotCalculator;

public class SotmCommands extends SubsystemBase {
      private final RobotContainer robot;

  public static final DAS das = new DAS();
  public static final ShotCalculator shotCalculator = new ShotCalculator();


  
  public SotmCommands(RobotContainer robot) {
    this.robot = robot;
  }


  public Command revSpeedFromDAS() {
    Command left = robot.shooterLeft.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return as.velocity;
    });
    Command right = robot.shooterRight.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return as.velocity;
    });

    return Commands.parallel(
        left,
        right
    // positionHoodFromDas()
    );
  }
}
