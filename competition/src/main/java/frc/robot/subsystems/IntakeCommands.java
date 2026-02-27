package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Degrees;

public class IntakeCommands extends SubsystemBase {

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private CommandSwerveDrivetrain swerve;


  public IntakeCommands(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.armSubsystem = armSubsystem;
  }
  
public Command armDown() {
    return armSubsystem.setAngle(Degrees.of(15)).alongWith(intakeSubsystem.outtakeDown())
            .until(() -> armSubsystem.isDown()).andThen(armSubsystem.set(-.1).alongWith(intakeSubsystem.stop()));
}

  }