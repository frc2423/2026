package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;

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

    private static final Pose2d leftTrenchPose = new Pose2d(3.5, 7.5, Rotation2d.fromDegrees(180));
    private static final Pose2d rightTrenchPose = new Pose2d(3.5, 0.5, Rotation2d.fromDegrees(180));
    private static final Pose2d hubPose = new Pose2d(3.5, 4, Rotation2d.fromDegrees(180));

    private static final Pose2d shootInFrontOfHubPose = new Pose2d(2.5, 4, Rotation2d.fromDegrees(-135));

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

        SmartDashboard.putData("setPoseToLeftTrench", resetRobotPose(leftTrenchPose));
        SmartDashboard.putData("setPoseToRightTrench", resetRobotPose(rightTrenchPose));
        SmartDashboard.putData("setPoseToHub", resetRobotPose(hubPose));

    }

    private Command resetRobotPose(Pose2d pose) {
        Command command = Commands.runOnce(() -> {
            drivetrain.resetPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
        });
        command.runsWhenDisabled();
        return command;
    }

    public Command startIntaking() {
        return Commands.deadline(
                arm.armDown(),
                intake.intake());
    }

    private Command driveToPose(Pose2d pose, boolean drivesShortestPath) {
        if (drivesShortestPath) {
            Path path = new Path(constraints, new Waypoint(pose));
            if (PoseTransformUtils.isRedAlliance()) {
                path.flip();
            }
            return bline.pathBuilder.build(path);
        }
        return bline.goToPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
    }

    public Command goToHubAndShoot() {
        return goToHubAndShoot(true);
    }

    public Command goToHubAndShoot(boolean drivesShortestPath) {
        Command driveToHub = driveToPose(shootInFrontOfHubPose, drivesShortestPath);
        return Commands.sequence(
                driveToHub,
                Commands.parallel(
                        shooter.prepareToShoot(),
                        Commands.waitSeconds(3).andThen(
                                shooter.spinFeeder(() -> feederSpeed))));
    }

    public Command centerAuto() {

        Command driveThroughTrench = driveShortestPath.driveShortestPath(
                flipPoseBasedOnRobotPose(new Pose2d(8, 3, Rotation2d.fromDegrees(90))));

        Command driveIntoFuel = driveShortestPath.driveShortestPath(
                flipPoseBasedOnRobotPose(new Pose2d(8, 1.3, Rotation2d.fromDegrees(90))));

        return Commands.sequence(
                startIntaking().withDeadline(Commands.sequence(driveThroughTrench, driveIntoFuel)),
                intake.stop(),
                goToHubAndShoot());

    }

    public Command outpostAuto() {
        return Commands.sequence(
                driveShortestPath.driveShortestPath(new Pose2d(0.4, 0.70, Rotation2d.fromDegrees(180))),
                Commands.waitSeconds(2),
                goToHubAndShoot());
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
                goToHubAndShoot());
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
