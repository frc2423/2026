package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
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
    private final Path shootToTrenchPath = new Path("Shoot-to-Trench");
    private final Path trenchToCollectPath = new Path("Trench-to-Collect");

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
        m_chooser.addOption("Center Twice Trench Auto with Long Shoot Delay",
                "Center Twice Trench Auto with Long Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto", "Center Twice Bump Auto");
        m_chooser.addOption("Custom Center Twice Bump Auto", "Custom Center Twice Bump Auto");
        m_chooser.addOption("Center Twice Bump Auto with Breif Delay", "Center Twice Bump Auto with Breif Delay");
        m_chooser.addOption("Center Twice Bump Auto with Shoot Delay", "Center Twice Bump Auto with Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto with Long Shoot Delay",
                "Center Twice Bump Auto with Long Shoot Delay");
        m_chooser.addOption("Center Twice Bump Auto with Outpost or Depot Delay",
                "Center Twice Bump Auto with Outpost or Depot Delay");
        m_chooser.addOption("Center Once Bump Depot or Outpost Auto", "Center Once Bump Depot or Outpost Auto");
        m_chooser.addOption("Center Once Bump Depot and Outpost Auto", "Center Once Bump Depot and Outpost Auto");
        m_chooser.addOption("Custom", "Custom");
        m_chooser.addOption("Center Once Bump Collect Auto", "Center Once Bump Collect Auto");
        m_chooser.addOption("Mechanical Advantage Delay", "Mechanical Advantage Delay");
        m_chooser.addOption("Custom Center Once Bump Collect Auto with Outpost or Depot Delay", "Custom Center Once Bump Collect Auto with Outpost or Depot Delay");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

        SmartDashboard.putData("setPoseToLeftTrench", resetRobotPose(leftTrenchPose));
        SmartDashboard.putData("setPoseToRightTrench", resetRobotPose(rightTrenchPose));
        SmartDashboard.putData("setPoseToHub", resetRobotPose(hubPose));

        NTHelper.setStringArray("/SmartDashboard/someStringArray", new String[]{"Default"});
        NTHelper.setDoubleArray("/SmartDashboard/setSomeNumberArray", new double[]{0.0});

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

    public Command goToHubAndShootUntil(BooleanSupplier condition) {
        Command driveToHub = driveToPose(shootInFrontOfHubPose);
        return Commands.sequence(
                Commands.deadline(driveToHub,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.shooterCommands.scoreUntil(condition),
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
                // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >=
                // 4)
                // .andThen(robot.intake.stop())),
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
                // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >=
                // 4)
                // .andThen(robot.intake.stop())),
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
                // Commands.waitUntil(() -> centerToBump.getCurrentTranslationElementIndex() >=
                // 4)
                // .andThen(robot.intake.stop())),
                robot.shooterCommands.scoreDeadline(4),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting(),
                // robot.shooterCommands.lookAtAngle(Rotation2d.fromDegrees(0),
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }

    public Timer timer = new Timer();

    // individual part commands to build a custom auto

    public Command shootDelay(double deadline) {
        return Commands.sequence(
                robot.shooterCommands.scoreUntil(() -> timer.hasElapsed(deadline-0.5)),
                robot.bline.pathBuilder.build(new Path(new Waypoint(robot.drivetrain.getPose().getTranslation(),
                        new Rotation2d(PoseTransformUtils.isRedAlliance() ? 0 : Math.PI)))));
    }

    public Command breifDelay(double deadline) {
        return Commands.waitUntil(() -> timer.hasElapsed(deadline));
    }

    public Command outpostOrDepot(double deadline) {
        boolean outpost = true;
        Path bumpToDepotOrOutpostPath = bumpToOutpostPath;

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            bumpToDepotOrOutpostPath = bumpToDepotPath;
            outpost = false;
            FloppingUtil.flopPath(shootToTrenchPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            bumpToDepotOrOutpostPath.flip();
            shootToTrenchPath.flip();
        }

        FollowPath bumpToDepotOrOutpost = robot.bline.pathBuilder.build(bumpToDepotOrOutpostPath);
        FollowPath shootToTrench = robot.bline.pathBuilder.build(shootToTrenchPath);

        Command bumpToDepotOrOutpostAndWait = bumpToDepotOrOutpost;
        if (outpost) {
            bumpToDepotOrOutpostAndWait = Commands.sequence(bumpToDepotOrOutpost, Commands.waitSeconds(2));
        }
        return Commands.sequence(
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                bumpToDepotOrOutpostAndWait,
                robot.intake.stop(),
                goToHubAndShootUntil(() -> timer.hasElapsed(deadline)),
                shootToTrench);
    }

    public Command centerTrench(double deadline) {
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

        return Commands.sequence(
                goToCenterAndIntake,
                Commands.deadline(
                        centerToLeftovers,
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.intake.stop(),
                robot.shooterCommands.scoreUntil(() -> timer.hasElapsed(deadline)),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

    public Command centerBump(double deadline) {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCenterPath);
            FloppingUtil.flopPath(centerToBumpPath);

        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToCenterPath.flip();
            centerToBumpPath.flip();
        }

        FollowPath trenchToCenter = robot.bline.pathBuilder.build(trenchToCenterPath);
        FollowPath centerToBump = robot.bline.pathBuilder.build(centerToBumpPath);

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
                        robot.shooterCommands.rev(() -> 3000.0)),
                robot.intake.stop(),
                robot.shooterCommands.scoreUntil(() -> timer.hasElapsed(deadline)),
                robot.shooterCommands.stopFeeding(),
                robot.shooterCommands.stopShooting());
    }

    public Command secondCenterTrench() {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToLeftoversPath);
            FloppingUtil.flopPath(leftoversToTrenchPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            trenchToLeftoversPath.flip();
            leftoversToTrenchPath.flip();
        }

        FollowPath trenchToLeftovers = robot.bline.pathBuilder.build(trenchToLeftoversPath);
        FollowPath leftoversToTrench = robot.bline.pathBuilder.build(leftoversToTrenchPath);

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
                trenchToLeftoversAndIntake,
                leftoversToTrenchAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }

    public Command secondCenterBump() {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(bumpToLeftoversPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {

            bumpToLeftoversPath.flip();
        }

        FollowPath bumpToLeftovers = robot.bline.pathBuilder.build(bumpToLeftoversPath);

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
                bumpToLeftoversAndIntake,
                robot.shooterCommands.scoreDeadline(10));
    }

    public Command collect() {
        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCollectPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {

            trenchToCollectPath.flip();
        }

        FollowPath trenchToCollect = robot.bline.pathBuilder.build(trenchToCollectPath);

        return Commands.sequence(
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                trenchToCollect);
    }

    public Command customAuto(String[] steps, double[] times) {
        if (steps.length != 3 || times.length != 2) {
            return Commands.none();
        }
        Command step1 = Commands.none();
        Command step2 = Commands.none();
        Command step3 = Commands.none();
        if (steps[0].equals("shoot")) {
            step1 = shootDelay(times[0]);
        } else if (steps[0].equals("brief")) {
            step1 = breifDelay(times[0]);
        } else if (steps[0].equals("outpost or depot")) {
            step1 = outpostOrDepot(times[0]);
        }
        if (steps[1].equals("trench")) {
            step2 = centerTrench(times[1]);
        } else if (steps[1].equals("bump")) {
            step2 = centerBump(times[1]);
        }
        if (steps[2].equals("trench")) {
            step3 = secondCenterTrench();
        } else if (steps[2].equals("bump")) {
            step3 = secondCenterBump();
        } else if (steps[2].equals("outpost or depot")) {
            step3 = outpostOrDepot(30);
        } else if (steps[2].equals("shoot")) {
            step3 = goToHubAndShoot();
        } else if (steps[2].equals("collect")) {
            step3 = collect();
        }
        return Commands.sequence(
                step1,
                step2,
                step3);
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Once Trench Auto")) {
            return centerOnceTrenchAuto();
        } else if (m_chooser.getSelected().equals("Outpost Auto")) {
            // return outpostAuto();
            return customAuto(new String[] { "outpost or depot", "", "" }, new double[] { 20, 20 });
        } else if (m_chooser.getSelected().equals("Outpost and Depot Auto")) {
            return outpostAndDepotAuto();
        } else if (m_chooser.getSelected().equals("Depot Auto")) {
            // return depotAuto();
            return customAuto(new String[] { "outpost or depot", "", "" }, new double[] { 20, 20 });

        } else if (m_chooser.getSelected().equals("Shoot Auto")) {
            // return shootAuto();
            return customAuto(new String[] { "", "", "shoot" }, new double[] { 20, 20 });
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto")) {
            // return centerTwiceTrenchAuto();
            return customAuto(new String[] { "", "trench", "trench" }, new double[] { 0, 10 });
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Breif Delay")) {
            // return centerTwiceTrenchAutoWithBreifDelay(1.5);
            return customAuto(new String[] { "brief", "trench", "trench" }, new double[] { 1.5, 11.5 });
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Shoot Delay")) {
            // return centerTwiceTrenchAutoWithShootDelay(3);
            return customAuto(new String[] { "shoot", "trench", "trench" }, new double[] { 3, 13 });
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Long Shoot Delay")) {
            // return centerTwiceTrenchAutoWithShootDelay(4);
            return customAuto(new String[] { "shoot", "trench", "trench" }, new double[] { 4.5, 10 });
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Outpost or Depot Delay")) {
            // return centerTwiceTrenchAuto();
            return customAuto(new String[] { "outpost or depot", "trench", "trench" }, new double[] { 6, 16 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto")) {
            return centerTwiceBumpAuto();
        } else if (m_chooser.getSelected().equals("Custom Center Twice Bump Auto")) {
            return customAuto(new String[] { "", "bump", "bump" }, new double[] { 0, 10 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Breif Delay")) {
            // return centerTwiceBumpAutoWithBreifDelay(1.5);
            return customAuto(new String[] { "brief", "bump", "bump" }, new double[] { 1.5, 11.5 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Shoot Delay")) {
            // return centerTwiceBumpAutoWithShootDelay(3);
            return customAuto(new String[] { "shoot", "bump", "bump" }, new double[] { 3, 13 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Long Shoot Delay")) {
            // return centerTwiceBumpAutoWithShootDelay(4);
            return customAuto(new String[] { "shoot", "bump", "bump" }, new double[] { 4.5, 14.5 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Outpost or Depot Delay")) {
            // return centerTwiceBumpAuto();
            return customAuto(new String[] { "outpost or depot", "bump", "bump" }, new double[] { 6, 16 });
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot or Outpost Auto")) {
            // return centerOnceBumpDepotOrOutpostAuto();
            return customAuto(new String[] { "", "bump", "outpost or depot" }, new double[] { 0, 12 });
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot and Outpost Auto")) {
            return centerOnceBumpDepotAndOutpostAuto();
        } else if (m_chooser.getSelected().equals("Custom")) {
            return customAuto(
                    NTHelper.getStringArray("/SmartDashboard/someStringArray", new String[]{"Default"}),
                    NTHelper.getDoubleArray("/SmartDashboard/setSomeNumberArray", new double[]{0.0}));
        } else if (m_chooser.getSelected().equals("Center Once Bump Collect Auto")) {
            return customAuto(new String[] { "", "bump", "collect" }, new double[] { 0, 12 });
        } else if (m_chooser.getSelected().equals("Mechanical Advantage Delay")) {
            return customAuto(new String[] { "shoot", "bump", "bump" }, new double[] { 5, 14 });
        } else if (m_chooser.getSelected().equals("Custom Center Once Bump Collect Auto with Outpost or Depot Delay")) {
            return customAuto(new String[] { "outpost or depot", "trench", "collect" }, new double[] { 5, 14 });
        } else{
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
