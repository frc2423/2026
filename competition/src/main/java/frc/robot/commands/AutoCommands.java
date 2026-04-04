package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
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

    private static final Pose2d shootInFrontOfHubPose = new Pose2d(2.5, 4, Rotation2d.fromDegrees(-180));

    private final Path trenchToCenterPath = new Path("Trench-to-Center");
    private final Path centerToTrenchPath = new Path("Center-to-Trench");
    private final Path trenchToLeftoversPath = new Path("Trench-to-Leftovers");
    private final Path leftoversToTrenchPath = new Path("Leftovers-to-Trench");
    private final Path centerToBumpPath = new Path("Center-to-Bump");
    private final Path bumpToLeftoversPath = new Path("Bump-to-Leftovers");
    private final Path centerToLeftoversPath = new Path("Center-to-Leftovers");
    private final Path bumpToOutpostPath = new Path("Bump-to-Outpost");
    private final Path bumpToDepotPath = new Path("Bump-to-Depot");
    private final Path trenchToOutpostPath = new Path("Trench-to-Outpost");
    private final Path outpostToDepotPath = new Path("Outpost-to-Depot");
    private final Path depotToDepotPath = new Path("Depot-to-Outpost");

    public AutoCommands(RobotContainer robot) {
        this.robot = robot;

        m_chooser.addOption("Center Once Trench Auto", "Center Once Trench Auto");
        m_chooser.addOption("Outpost Auto", "Outpost Auto");
        m_chooser.addOption("Outpost and Depot Auto", "Outpost and Depot Auto");
        m_chooser.addOption("Depot Auto", "Depot Auto");
        m_chooser.addOption("Shoot Auto", "Shoot Auto");
        m_chooser.addOption("Center Twice Trench Auto", "Center Twice Trench Auto");
        m_chooser.addOption("Center Twice Trench Auto with Breif Delay", "Center Twice Trench Auto with Breif Delay");
        m_chooser.addOption("Center Twice Trench Auto with Shoot Delay", "Center Twice Trench Auto with Shoot Delay");
        m_chooser.addOption("Center Twice Trench Auto with Long Shoot Delay", "Center Twice Trench Auto with Long Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto", "Center Twice Bump Auto");
        m_chooser.addOption("Center Twice Bump Auto with Breif Delay", "Center Twice Bump Auto with Breif Delay");
        m_chooser.addOption("Center Twice Bump Auto with Shoot Delay", "Center Twice Bump Auto with Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto with Long Shoot Delay", "Center Twice Bump Auto with Long Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto with Outpost or Depot Delay", "Center Twice Bump Auto with Outpost or Depot Delay");
        m_chooser.addOption("Center Once Bump Depot or Outpost Auto", "Center Once Bump Depot or Outpost Auto");
        m_chooser.addOption("Center Once Bump Depot and Outpost Auto", "Center Once Bump Depot and Outpost Auto");
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

    private Command driveToPose(Pose2d pose) {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            pose = FloppingUtil.flop(pose);
        }
        return robot.bline.goToPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
    }

    public Command goToHubAndShoot() {
        Command driveToHub = driveToPose(shootInFrontOfHubPose);
        return Commands.sequence(
                Commands.deadline(driveToHub,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(20),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

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
                        robot.intakeCommands.armDown(),
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intake.intake()));

        // Command intakeInCenterMore = Commands.deadline(centerToLeftovers,
        // Commands.sequence(
        // Commands.waitUntil(() ->
        // centerToLeftovers.getCurrentTranslationElementIndex() >= 8),
        // robot.intake.stop())
        // );

        return Commands.sequence(
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

    public Command centerOnceBumpAuto() {

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToLeftovers,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.intake.stop(),
                robot.shooterCommands.scoreDeadline(20),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

    public Command centerOnceBumpDepotOrOutpostAuto() {
        boolean outpost = true;
        Path bumpToDepotOrOutpostPath = bumpToOutpostPath;

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            bumpToDepotOrOutpostPath = bumpToDepotPath;
            outpost = false;
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToDepotOrOutpostPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToDepotOrOutpost = robot.bline.pathBuilder.build(bumpToDepotOrOutpostPath);

        Command bumpToDepotOrOutpostAndWait = bumpToDepotOrOutpost;
        if (outpost) {
            bumpToDepotOrOutpostAndWait = Commands.sequence(bumpToDepotOrOutpost, Commands.waitSeconds(2));
        }

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 2)
                                .andThen(robot.intake.stop()),
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(outpost ? 3 : 5),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                bumpToDepotOrOutpostAndWait,
                robot.intake.stop(),
                goToHubAndShoot());
    }

    public Command centerOnceBumpDepotAndOutpostAuto() {
        boolean outpost = true;
        Path bumpToDepotOrOutpostPath = bumpToOutpostPath;
        Path depotOrOutpostToOtherPath = outpostToDepotPath;

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            bumpToDepotOrOutpostPath = bumpToDepotPath;
            depotOrOutpostToOtherPath = depotToDepotPath;
            outpost = false;
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToDepotOrOutpostPath.flip();
            depotOrOutpostToOtherPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToDepotOrOutpost = robot.bline.pathBuilder.build(bumpToDepotOrOutpostPath);
        FollowPath depotOrOutpostToOther = robot.bline.pathBuilder.build(depotOrOutpostToOtherPath);

        Command bumpToDepotOrOutpostAndWait = bumpToDepotOrOutpost;
        Command depotOrOutpostToOtherAndWait = depotOrOutpostToOther;
        if (outpost) {
            bumpToDepotOrOutpostAndWait = Commands.sequence(bumpToDepotOrOutpost, Commands.waitSeconds(2));
        } else {
            depotOrOutpostToOtherAndWait = Commands.sequence(depotOrOutpostToOther, Commands.waitSeconds(2));
        }

        Command goToCenterAndIntake = Commands.deadline(
                trenchToCenter,
                Commands.sequence(
                        Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                        robot.intakeCommands.armDown(),
                        robot.intake.intake()));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 2)
                                .andThen(robot.intake.stop()),
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreDeadline(3),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                bumpToDepotOrOutpostAndWait,
                depotOrOutpostToOtherAndWait,
                goToHubAndShoot());
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

    public Command centerTwiceTrenchAutoWithBreifDelay(double seconds) {

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
                Commands.waitSeconds(seconds),
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

    public Command centerTwiceTrenchAutoWithShootDelay(double seconds) {

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
                robot.shooterCommands.scoreDeadline(seconds),
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
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.sequence(
                Commands.deadline(
                        trenchToCenter,
                        Commands.sequence(
                                robot.intakeCommands.armDown(),
                                Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                                robot.intake.intake()))
                        .withTimeout(5),
                robot.intake.intake());

        Command bumpToLeftoversAndIntake = Commands.deadline(
                bumpToLeftovers,
                Commands.sequence(
                        robot.intakeCommands.armDown(),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 6),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 12),
                        robot.shooterCommands.rev(() -> 3000.0)));

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        robot.shooterCommands.rev(() -> 3000.0)),
                        // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 4)
                        //         .andThen(robot.intake.stop())),
                robot.shooterCommands.scoreDeadline(4.5),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }

    // 1.5 second delay
    public Command centerTwiceBumpAutoWithBreifDelay(double seconds) {

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.sequence(
                Commands.deadline(
                        trenchToCenter,
                        Commands.sequence(
                                robot.intakeCommands.armDown(),
                                Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                                robot.intake.intake()))
                        .withTimeout(5),
                robot.intake.intake());

        Command bumpToLeftoversAndIntake = Commands.deadline(
                bumpToLeftovers,
                Commands.sequence(
                        robot.intakeCommands.armDown(),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 6),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 12),
                        robot.shooterCommands.rev(() -> 3000.0)));

        return Commands.sequence(
                Commands.waitSeconds(seconds),
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        robot.shooterCommands.rev(() -> 3000.0)),
                        // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 4)
                        //         .andThen(robot.intake.stop())),
                robot.shooterCommands.scoreDeadline(4.5),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }

    // 3 second delay
    public Command centerTwiceBumpAutoWithShootDelay(double seconds) {

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
            bumpToLeftoversPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);
        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

        Command goToCenterAndIntake = Commands.sequence(
                Commands.deadline(
                        trenchToCenter,
                        Commands.sequence(
                                robot.intakeCommands.armDown(),
                                Commands.waitUntil(() -> trenchToCenter.getCurrentTranslationElementIndex() >= 4),
                                robot.intake.intake()))
                        .withTimeout(5),
                robot.intake.intake());

        Command bumpToLeftoversAndIntake = Commands.deadline(
                bumpToLeftovers,
                Commands.sequence(
                        robot.intakeCommands.armDown(),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 6),
                        robot.intake.intake(),
                        Commands.waitUntil(() -> bumpToLeftovers.getCurrentTranslationElementIndex() >= 12),
                        robot.shooterCommands.rev(() -> 3000.0)));

        return Commands.sequence(
                robot.shooterCommands.scoreDeadline(seconds),
                goToCenterAndIntake,
                Commands.deadline(
                        centerToBump,
                        robot.shooterCommands.rev(() -> 3000.0)),
                        // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >= 4)
                        //         .andThen(robot.intake.stop())),
                robot.shooterCommands.scoreDeadline(4),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
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
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Breif Delay")) {
            return centerTwiceTrenchAutoWithBreifDelay(1.5);
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Shoot Delay")) {
            return centerTwiceTrenchAutoWithShootDelay(3);
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Long Shoot Delay")) {
            return centerTwiceTrenchAutoWithShootDelay(4);
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Outpost or Depot Delay")) {
            return centerTwiceTrenchAuto();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto")) {
            return centerTwiceBumpAuto();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Breif Delay")) {
            return centerTwiceBumpAutoWithBreifDelay(1.5);
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Shoot Delay")) {
            return centerTwiceBumpAutoWithShootDelay(3);
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Long Shoot Delay")) {
            return centerTwiceBumpAutoWithShootDelay(4);
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Outpost or Depot Delay")) {
            return centerTwiceBumpAuto();
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot or Outpost Auto")) {
            return centerOnceBumpDepotOrOutpostAuto();
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot and Outpost Auto")) {
            return centerOnceBumpDepotAndOutpostAuto();
        } else {
            return Commands.none();
        }
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
