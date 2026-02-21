package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

import static edu.wpi.first.units.Units.Degrees;

import javax.lang.model.element.Element;

import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.epilogue.Logged;

public class AutoCommands {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final DriveShortestPath driveShortestPath;
    private final ShooterCommands shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final BLine bline;
    private final PathConstraints constraints = new PathConstraints()
            .setMaxVelocityMetersPerSec(2);
    public final double feederSpeed = 10;

    // @Logged
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public AutoCommands(ArmSubsystem arm, DriveShortestPath driveShortestPath, IntakeSubsystem intake,
            ShooterCommands shooter, CommandSwerveDrivetrain drivetrain, BLine bline) {
        this.intake = intake;
        this.driveShortestPath = driveShortestPath;
        this.arm = arm;
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.bline = bline;

        m_chooser.addOption("Center Piece Auto", "Center Piece Auto");
        m_chooser.addOption("Outpost Auto", "Outpost Auto");
        m_chooser.addOption("Outpost and Depot Auto", "Outpost and Depot Auto");
        m_chooser.addOption("Depot Auto", "Depot Auto");
        m_chooser.addOption("Shoot Auto", "Shoot Auto");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

    }

    public Command startIntaking() {
        return Commands.deadline(
                arm.armDown(),
                intake.intake());
    }

    public Command goToHubAndShoot() {
        return goToHubAndShoot(drivetrain.getPose());
    }

    public Command goToHubAndShoot(Pose2d lastPose2d) {
        return Commands.sequence(
                driveShortestPath.driveShortestPath(
                        flipPoseBasedOnRobotPose(new Pose2d(2.5, 3, Rotation2d.fromDegrees(-135)), lastPose2d)),
                shooter.prepareToShoot(),
                shooter.spinFeeder(() -> feederSpeed));
    }

    public Command centerAuto() {
        return Commands.sequence(
                startIntaking().withDeadline(Commands.sequence(driveShortestPath.driveShortestPath(
                        flipPoseBasedOnRobotPose(new Pose2d(8, 3, Rotation2d.fromDegrees(90)))),
                        driveShortestPath.driveShortestPath(
                                flipPoseBasedOnRobotPose(new Pose2d(8, 1.3, Rotation2d.fromDegrees(90)))))),
                intake.stop(),
                goToHubAndShoot());

    }

    public Command outpostAuto() {
        return Commands.sequence(
                driveShortestPath.driveShortestPath(new Pose2d(0.4, 0.70, Rotation2d.fromDegrees(180))),
                Commands.waitSeconds(2),
                goToHubAndShoot(new Pose2d(0.4, 0.70, Rotation2d.fromDegrees(180))));
    }

    public Command outpostAndDepotAuto() {
        Path path = new Path(constraints,
                new Waypoint(new Pose2d(2.25, 2, Rotation2d.fromDegrees(180))),
                new Waypoint(new Pose2d(2.25, 5, Rotation2d.fromDegrees(180))),
                new Waypoint(new Pose2d(1.25, 6, Rotation2d.fromDegrees(180))));
        if (PoseTransformUtils.isRedAlliance()) {
            path.flip();
        }
        return Commands.sequence(
                bline.pathBuilder.build(new Path(constraints,
                        new Waypoint(new Pose2d(0.4, 0.70, Rotation2d.fromDegrees(180))))),
                Commands.waitSeconds(2),
                bline.pathBuilder.build(path),
                startIntaking(),
                driveShortestPath.driveShortestPath(new Pose2d(0.4, 6, Rotation2d.fromDegrees(180))),
                intake.stop(),
                goToHubAndShoot(new Pose2d(0.4, 6, Rotation2d.fromDegrees(180))));
    }

    public Command depotAuto() {
        return Commands.sequence(
                startIntaking().withDeadline(
                        driveShortestPath.driveShortestPath(new Pose2d(1.25, 6, Rotation2d.fromDegrees(180)))),
                driveShortestPath.driveShortestPath(new Pose2d(0.4, 6, Rotation2d.fromDegrees(180))),
                intake.stop(),
                goToHubAndShoot());
    }

    public Command shootAuto() {
        return goToHubAndShoot();

        // Commands.sequence(shooter.prepareToShoot(),
        // shooter.spinFeeder(() -> feederSpeed));
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Piece Auto")) {
            return centerAuto();
        } else if (m_chooser.getSelected().equals("Outpost Auto")) {
            return outpostAuto();
        } else if (m_chooser.getSelected().equals("Outpost and Depot Auto")) {
            return outpostAndDepotAuto();
        } else if (m_chooser.getSelected().equals("Depot Auto")) {
            return depotAuto();
        } else if (m_chooser.getSelected().equals("Shoot Auto")) {
            return shootAuto();
        } else {
            return Commands.none();
        }
    }

    private Pose2d flipPoseBasedOnRobotPose(Pose2d unflippedPose2d) {
        return flipPoseBasedOnRobotPose(unflippedPose2d, drivetrain.getPose());
    }

    private Pose2d flipPoseBasedOnRobotPose(Pose2d unflippedPose2d, Pose2d lastPose2d) {
        boolean needsToFlip = (lastPose2d.getY() >= FieldConstants.LinesHorizontal.center) != (unflippedPose2d
                .getY() >= FieldConstants.LinesHorizontal.center);
        if (PoseTransformUtils.isRedAlliance()) {
            needsToFlip = !needsToFlip;
        }
        double flippedY = PoseTransformUtils.FIELD_WIDTH_METERS - unflippedPose2d.getY();
        Rotation2d flippedRotation2d = new Rotation2d((2 * Math.PI) - unflippedPose2d.getRotation().getRadians());
        Pose2d flippedPose2d = needsToFlip ? new Pose2d(unflippedPose2d.getX(), flippedY, flippedRotation2d)
                : unflippedPose2d;
        return flippedPose2d;
    }
}
