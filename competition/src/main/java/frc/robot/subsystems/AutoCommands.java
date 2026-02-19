package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;

import static edu.wpi.first.units.Units.Degrees;

import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.epilogue.Logged;

public class AutoCommands {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final DriveShortestPath driveShortestPath;
    private final ShooterCommands shooter;
    private final CommandSwerveDrivetrain drivetrain;
    public final double feederSpeed = 10;

    // @Logged
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public AutoCommands(ArmSubsystem arm, DriveShortestPath driveShortestPath, IntakeSubsystem intake,
            ShooterCommands shooter, CommandSwerveDrivetrain drivetrain) {
        this.intake = intake;
        this.driveShortestPath = driveShortestPath;
        this.arm = arm;
        this.shooter = shooter;
        this.drivetrain = drivetrain;

        m_chooser.addOption("Center Piece Auto", "Center Piece Auto");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

    }

    public Command partOfAuto() {
        return Commands.deadline(
                 driveShortestPath.driveShortestPath(flipPoseBasedOnRobotPose(new Pose2d(8, 3, new Rotation2d().fromDegrees(90)))),
                 arm.setAngle(Degrees.of(10)),
                 intake.intake());
    }

    public Command centerAuto() {
        return Commands.sequence(
                driveShortestPath.driveShortestPath(flipPoseBasedOnRobotPose(new Pose2d(8, 1.3, new Rotation2d().fromDegrees(90)))),
                partOfAuto(),
                intake.stop(),
                driveShortestPath.driveShortestPath(flipPoseBasedOnRobotPose(new Pose2d(2.5, 3, new Rotation2d().fromDegrees(-135)))),
                shooter.prepareToShoot(),
                shooter.spinFeeder(() -> feederSpeed));
    }

    public Command shoot() {
        return Commands.sequence(shooter.prepareToShoot(),
        shooter.spinFeeder(() -> feederSpeed));
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Piece Auto")) {
            return Commands.sequence(
                centerAuto().withTimeout(14),
                centerAuto());
        } else {
            return Commands.none();
        }
    }

    // NAME DESPERATLY NEEDS TO BE CHNAGHED
    // (this function flips the entered pose based on the half of the field the robot is on, (not red or blue, but left or right). It's made for use in autos.)
    private Pose2d flipPoseBasedOnRobotPose(Pose2d pose2d) {
        boolean needsToFlip = (drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) != (pose2d.getY() >= FieldConstants.LinesHorizontal.center);
        if (PoseTransformUtils.isRedAlliance()) {
            needsToFlip = !needsToFlip;
        }
        double flippedY = PoseTransformUtils.FIELD_WIDTH_METERS - pose2d.getY();
        Rotation2d flippedRotation2d = new Rotation2d((2*Math.PI) - pose2d.getRotation().getRadians());
        Pose2d flippedPose2d = needsToFlip ? new Pose2d(pose2d.getX(),flippedY,flippedRotation2d) : pose2d;
        return flippedPose2d;
    }
}
