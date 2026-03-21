package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.FloppingUtil;

public class AutoCommands {
    private final RobotContainer robot;

    private final PathConstraints constraints = new PathConstraints()
            .setMaxVelocityMetersPerSec(1);
    public final double feederSpeed = 0.25 * 10;

    // @Logged
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private static final Pose2d leftTrenchPose = new Pose2d(3.5, 7.5, Rotation2d.fromDegrees(180));
    private static final Pose2d rightTrenchPose = new Pose2d(3.5, 0.5, Rotation2d.fromDegrees(180));
    private static final Pose2d hubPose = new Pose2d(3.5, 4, Rotation2d.fromDegrees(180));

    private static final Pose2d shootInFrontOfHubPose = new Pose2d(2.5, 4, Rotation2d.fromDegrees(-135));

    public AutoCommands(RobotContainer robot) {
        this.robot = robot;

        m_chooser.addOption("Center Once Trench Auto", "Center Once Trench Auto");
        m_chooser.addOption("Outpost Auto", "Outpost Auto");
        m_chooser.addOption("Outpost and Depot Auto", "Outpost and Depot Auto");
        m_chooser.addOption("Depot Auto", "Depot Auto");
        m_chooser.addOption("Shoot Auto", "Shoot Auto");
        m_chooser.addOption("Center Twice Trench Auto", "Center Twice Trench Auto");
        m_chooser.addOption("Center Twice Bump Auto", "Center Twice Bump Auto");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

        SmartDashboard.putData("setPoseToLeftTrench", resetRobotPose(leftTrenchPose));
        SmartDashboard.putData("setPoseToRightTrench", resetRobotPose(rightTrenchPose));
        SmartDashboard.putData("setPoseToHub", resetRobotPose(hubPose));

    }

    private Command resetRobotPose(Pose2d pose) {
        Command command = Commands.runOnce(() -> {
            robot.drivetrain.resetPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
        });
        command.runsWhenDisabled();
        return command;
    }

    public Command startIntaking() {
        return Commands.sequence(
                robot.intakeCommands.armDown(),
                robot.intake.intake());
    }

    private Command driveToPose(Pose2d pose, boolean drivesShortestPath) {
        if (drivesShortestPath) {
            Path path = new Path(constraints, new Waypoint(pose));
            if (PoseTransformUtils.isRedAlliance()) {
                path.flip();
            }
            return robot.bline.pathBuilder.build(path);
        }
        return robot.bline.goToPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
    }

    public Command goToHubAndShoot() {
        return goToHubAndShoot(true);
    }

    public Command goToHubAndShoot(boolean drivesShortestPath) {
        Command driveToHub = driveToPose(shootInFrontOfHubPose, drivesShortestPath);
        return Commands.sequence(
                driveToHub,
                Commands.parallel(
                        robot.shooterCommands.prepareToShoot(),
                        Commands.waitSeconds(3).andThen(
                                robot.shooterCommands.feed())));
    }

    Path trenchToCenterPath = new Path("Trench-to-Center");
    Path centerToLeftoversPath = new Path("Center-to-Leftovers");

    public Command centerOnceTrenchAuto() {

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToLeftovers = robot.bline.pathBuilder.build(centerToLeftoversPath);

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        // Command intakeInCenterMore = Commands.deadline(centerToLeftovers,
        // Commands.sequence(
        // Commands.waitUntil(() ->
        // centerToLeftovers.getCurrentTranslationElementIndex() >= 8),
        // robot.intake.stop())
        // );

        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.println("TIME 3: " + Timer.getTimestamp());

                }),
                goToCenterAndIntake,
                Commands.print("CENTER TO TRENCH"),
                Commands.deadline(
                        centerToLeftovers,
                        robot.shooterCommands.rev(() -> 3000.0)),
                Commands.print("SCORE DEADLINE"),
                robot.intake.stop(),
                robot.shooterCommands.scoreDeadline(20),
                Commands.print("STOP FEEDING"),
                robot.shooterCommands.stopFeeding(),
                Commands.print("STOP SHOOTING"),
                robot.shooterCommands.stopShooting());
    }

    public Command outpostAuto() {
        Path outpost = new Path(constraints,
                new Waypoint(new Pose2d(1.0, 0.70, Rotation2d.fromDegrees(180))),
                new Waypoint(new Pose2d(0.4, 0.70, Rotation2d.fromDegrees(180))));
        if (PoseTransformUtils.isRedAlliance()) {
            outpost.flip();
        }
        return Commands.sequence(
                robot.bline.pathBuilder.build(outpost),
                Commands.waitSeconds(2),
                goToHubAndShoot());
    }

    public Command outpostAndDepotAuto() {
        Path trenchToOutpostPath = new Path("Trench-to-Outpost");
        Path outpostToDepotPath = new Path("Outpost-to-Depot");

        if (PoseTransformUtils.isRedAlliance()) {
            trenchToOutpostPath.flip();
            outpostToDepotPath.flip();
        }

        FollowPath trenchToOutpost = robot.bline.pathBuilder.build(trenchToOutpostPath);
        FollowPath outpostToDepot = robot.bline.pathBuilder.build(outpostToDepotPath);

        return Commands.sequence(
                trenchToOutpost,
                Commands.waitSeconds(2),
                Commands.deadline(
                        outpostToDepot,
                        Commands.waitUntil(() -> outpostToDepot.getCurrentTranslationElementIndex() >= 4)
                                .andThen(startIntaking())),
                robot.intake.stop(),
                goToHubAndShoot());
    }

    public Command depotAuto() {
        Pose2d depot = new Pose2d(1.25, 6, Rotation2d.fromDegrees(180));
        Pose2d shoot = new Pose2d(0.4, 6, Rotation2d.fromDegrees(180));
        if (PoseTransformUtils.isRedAlliance()) {
            FlippingUtil.flipFieldPose(depot);
            FlippingUtil.flipFieldPose(shoot);
        }
        return Commands.sequence(
                robot.driveShortestPath.driveShortestPath(depot),
                startIntaking(),
                robot.driveShortestPath.driveShortestPath(shoot),
                robot.intake.stop(),
                goToHubAndShoot());
    }

    public Command shootAuto() {
        return goToHubAndShoot();

        // Commands.sequence(shooter.prepareToShoot(),
        // shooter.spinFeeder(() -> feederSpeed));
    }

    public Command centerTwiceTrenchAuto() {

        Path trenchToCenterPath = new Path("Trench-to-Center");
        Path centerToTrenchPath = new Path("Center-to-Trench");
        Path trenchToLeftoversPath = new Path("Trench-to-Leftovers");
        Path leftoversToTrenchPath = new Path("Leftovers-to-Trench");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToTrenchPath);
            FloppingUtil.flopPath(trenchToLeftoversPath);
            FloppingUtil.flopPath(leftoversToTrenchPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToTrenchPath.flip();
            trenchToLeftoversPath.flip();
            leftoversToTrenchPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToTrench = robot.bline.pathBuilder.build(centerToTrenchPath);
        FollowPath trenchToLeftovers = robot.bline.pathBuilder.build(trenchToLeftoversPath);
        FollowPath leftoversToTrench = robot.bline.pathBuilder.build(leftoversToTrenchPath);

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        Command trenchToLeftoversAndIntake = Commands.deadline(
                trenchToLeftovers,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToLeftovers.getCurrentTranslationElementIndex() >= 2),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        Command leftoversToTrenchAndIntake = Commands.deadline(
                leftoversToTrench,
                Commands.waitUntil(() -> leftoversToTrench.getCurrentTranslationElementIndex() >= 3)
                        .andThen(robot.intake.stop()));

        return Commands.sequence(
                goToCenterAndIntake,
                robot.intake.stop(),
                Commands.print("CENTER TO TRENCH"),
                Commands.deadline(
                        centerToTrench,
                        robot.shooterCommands.rev(() -> 3000.0)),
                Commands.print("SCORE DEADLINE"),
                robot.shooterCommands.scoreDeadline(3),
                Commands.print("STOP FEEDING"),
                robot.shooterCommands.stopFeeding(),
                Commands.print("STOP SHOOTING"),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0)),
                Commands.print("THE REST"),
                trenchToLeftoversAndIntake,
                leftoversToTrenchAndIntake,
                robot.shooterCommands.scoreDeadline(3));
    }

    public Command centerTwiceBumpAuto() {

        Path trenchToCenterPath = new Path("Trench-to-Center");
        Path centerToBumpPath = new Path("Center-to-Bump");
        Path bumpToLeftoversPath = new Path("Bump-to-Leftovers");
        Path leftoversToBumpPath = new Path("Leftovers-to-Bump");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
            FloppingUtil.flopPath(leftoversToBumpPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToLeftoversPath.flip();
            leftoversToBumpPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);
        FollowPath leftoversToBump = robot.bline.pathBuilder.build(leftoversToBumpPath);

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        Command bumpToLeftoversAndIntake = Commands.deadline(
                bumpToLeftovers,
                Commands.sequence(
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 2),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        Command leftoversToBumpAndIntake = Commands.deadline(
                leftoversToBump,
                Commands.waitUntil(() -> leftoversToBump.getCurrentTranslationElementIndex() >= 3)
                        .andThen(robot.intake.stop()));

        return Commands.sequence(
                goToCenterAndIntake,
                robot.intake.stop(),
                Commands.deadline(
                        centerToBump,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(3),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                leftoversToBumpAndIntake,
                robot.shooterCommands.scoreDeadline(3));
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Once Trench Auto")) {
            return centerOnceTrenchAuto();
        } else if (m_chooser.getSelected().equals("Outpost Auto")) {
            return outpostAuto();
        } else if (m_chooser.getSelected().equals("Outpost and Depot Auto")) {
            return outpostAndDepotAuto();
        } else if (m_chooser.getSelected().equals("Depot Auto")) {
            return depotAuto();
        } else if (m_chooser.getSelected().equals("Shoot Auto")) {
            return shootAuto();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto")) {
            return centerTwiceTrenchAuto();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto")) {
            return centerTwiceBumpAuto();
        } else {
            return Commands.none();
        }
    }

    public String getSelectedAuto() {
        String autoSelection = m_chooser.getSelected();
        if (autoSelection == null) {
            return "";
        }
        return autoSelection;
    }

    private String previousAutoSelection = "";

    public Trigger autoSelectionChange() {
        return new Trigger(() -> {
            String autoSelection = getSelectedAuto();
            boolean hasChanged = !previousAutoSelection.equals(autoSelection);
            previousAutoSelection = autoSelection;
            return hasChanged;
        });
    }

    private Pose2d flipPoseBasedOnRobotPose(Pose2d unflippedPose2d) {
        return flipPoseBasedOnRobotPose(unflippedPose2d, robot.drivetrain.getPose());
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
