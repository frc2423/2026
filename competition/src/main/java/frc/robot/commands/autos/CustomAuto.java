package frc.robot.commands.autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoPaths;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.FloppingUtil;

/**
 * An auto built from 3 steps chosen on the dashboard. The deadlines in times are
 * measured from the start of auto using the shared auto timer.
 */
public class CustomAuto extends AutoRoutine {
    private final Timer timer;
    private final String[] steps;
    private final double[] times;

    public CustomAuto(RobotContainer robot, Timer timer, String[] steps, double[] times) {
        super(robot);
        this.timer = timer;
        this.steps = steps;
        this.times = times;
    }

    @Override
    public Command build() {
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

    // individual part commands to build a custom auto

    private Command shootDelay(double deadline) {
        return Commands.sequence(
                robot.shooterCommands.scoreUntil(() -> timer.hasElapsed(deadline - 0.5)),
                robot.shooterCommands.lookAtAngle(new Rotation2d(PoseTransformUtils.isRedAlliance() ? Math.PI : 0))
                        .withTimeout(1),
                robot.shooterCommands.stopShooting(),
                robot.shooterCommands.stopFeeding(),
                robot.intakeCommands.armDown());
    }

    private Command breifDelay(double deadline) {
        return Commands.waitUntil(() -> timer.hasElapsed(deadline));
    }

    private Command outpostOrDepot(double deadline) {
        Path shootToTrenchPath = AutoPaths.getPath("Shoot-to-Trench");

        boolean outpost = true;
        Path bumpToDepotOrOutpostPath = AutoPaths.getPath("Bump-to-Outpost");

        if (!isOutpostSide()) {
            bumpToDepotOrOutpostPath = AutoPaths.getPath("Bump-to-Depot");
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

    private Command centerTrench(double deadline) {
        Path trenchToCenterPath = AutoPaths.getPath("Trench-to-Center");
        Path centerToLeftoversPath = AutoPaths.getPath("Center-to-Leftovers");

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

    private Command centerBump(double deadline) {
        Path trenchToCenterPath = AutoPaths.getPath("Trench-to-Center");
        Path centerToBumpPath = AutoPaths.getPath("Center-to-Bump");

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

    private Command secondCenterTrench() {
        Path trenchToLeftoversPath = AutoPaths.getPath("Trench-to-Leftovers");
        Path leftoversToTrenchPath = AutoPaths.getPath("Leftovers-to-Trench");

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

    private Command secondCenterBump() {
        Path bumpToLeftoversPath = AutoPaths.getPath("Bump-to-Leftovers");

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

    private Command collect() {
        Path trenchToCollectPath = AutoPaths.getPath("Trench-to-Collect");
        Path shootToTrenchPath = AutoPaths.getPath("Shoot-to-Trench");

        if ((robot.drivetrain.getPose().getY() >= FieldConstants.LinesHorizontal.center) == !PoseTransformUtils
                .isRedAlliance()) {
            FloppingUtil.flopPath(trenchToCollectPath);
            FloppingUtil.flopPath(shootToTrenchPath);
        }
        if (PoseTransformUtils.isRedAlliance()) {
            shootToTrenchPath.flip();
            trenchToCollectPath.flip();
        }

        FollowPath trenchToCollect = robot.bline.pathBuilder.build(trenchToCollectPath);
        FollowPath shootToTrench = robot.bline.pathBuilder.build(shootToTrenchPath);

        return Commands.sequence(
                shootToTrench,
                robot.intakeCommands.armDown(),
                robot.intake.intake(),
                trenchToCollect);
    }

    public static boolean isOutpostSide() {
        return NTHelper.getString("OutpostOrDepotSide", "outpost").equals("oupost");
    }

    private static void addAllDoubleArray(ArrayList<Double> list, double[] array) {
        for (double num : array) {
            list.add(num);
        }
    }

    /** Flattened [x, y, degrees, ...] poses of the custom auto's steps, for previewing on the dashboard. */
    public static double[] getCombinedPath(String[] steps) {
        ArrayList<Double> poses = new ArrayList<>();
        double[] startIndexes = new double[3];
        if (steps.length != 3) {
            return new double[0];
        }
        startIndexes[0] = 0;
        if (steps[0].equals("outpost or depot")) {
            if (isOutpostSide()) {
                addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Bump-to-Outpost")));
            } else {
                addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Bump-to-Depot")));
            }
            poses.add(shootInFrontOfHubPose.getX());
            poses.add(shootInFrontOfHubPose.getY());
            poses.add(shootInFrontOfHubPose.getRotation().getDegrees());
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Shoot-to-Trench")));
        }
        startIndexes[1] = poses.size();
        if (steps[1].equals("trench")) {
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Trench-to-Center")));
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Center-to-Leftovers")));
        } else if (steps[1].equals("bump")) {
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Trench-to-Center")));
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Center-to-Bump")));
        }
        startIndexes[2] = poses.size();
        if (steps[2].equals("trench")) {
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Trench-to-Leftovers")));
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Leftovers-to-Trench")));
        } else if (steps[2].equals("bump")) {
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Bump-to-Leftovers")));
        } else if (steps[2].equals("outpost or depot")) {
            if (isOutpostSide()) {
                addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Bump-to-Outpost")));
            } else {
                addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Bump-to-Depot")));
            }
            poses.add(shootInFrontOfHubPose.getX());
            poses.add(shootInFrontOfHubPose.getY());
            poses.add(shootInFrontOfHubPose.getRotation().getDegrees());
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Shoot-to-Trench")));
        } else if (steps[2].equals("shoot")) {
            poses.add(shootInFrontOfHubPose.getX());
            poses.add(shootInFrontOfHubPose.getY());
            poses.add(shootInFrontOfHubPose.getRotation().getDegrees());
        } else if (steps[2].equals("collect")) {
            addAllDoubleArray(poses, AutoPaths.getDoubleArrayFromPath(AutoPaths.getPath("Trench-to-Collect")));
        }
        NTHelper.setDoubleArray("/SmartDashboard/startIndexes", startIndexes);
        double[] posesArray = new double[poses.size()];
        for (int j = 0; j < poses.size(); j++) {
            posesArray[j] = poses.get(j);
        }
        return posesArray;
    }
}
