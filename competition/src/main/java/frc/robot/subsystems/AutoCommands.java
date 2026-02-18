package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;

public class AutoCommands {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final DriveShortestPath driveShortestPath;
    private final ShooterCommands shooter;
    public final double feederSpeed = 10;

    // @Logged
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public AutoCommands(ArmSubsystem arm, DriveShortestPath driveShortestPath, IntakeSubsystem intake,
            ShooterCommands shooter) {
        this.intake = intake;
        this.driveShortestPath = driveShortestPath;
        this.arm = arm;
        this.shooter = shooter;

        m_chooser.addOption("Center Piece Auto", "Center Piece Auto");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

    }

    public Command partOfAuto() {
        return Commands.deadline(
                 driveShortestPath.driveShortestPath(new Pose2d(8, 3, new Rotation2d().fromDegrees(90))),
                 arm.setAngle(Degrees.of(10)),
                 intake.intake());
    }

    public Command centerAuto() {
        return Commands.sequence(
                driveShortestPath.driveShortestPath(new Pose2d(8, 1.3, new Rotation2d().fromDegrees(90))),
                partOfAuto(),
                intake.stop(),
                driveShortestPath.driveShortestPath(new Pose2d(2.5, 3, new Rotation2d().fromDegrees(-135))),
                shooter.prepareToShoot(),
                shooter.spinFeeder(() -> feederSpeed));
    }

    public Command shoot() {
        return Commands.sequence(shooter.prepareToShoot(),
        shooter.spinFeeder(() -> feederSpeed));
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Piece Auto")) {
            return centerAuto();
        } else {
            return Commands.none();
        }
    }
}
