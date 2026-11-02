package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.commands.autos.CenterOnceBumpDepotAndOutpostAuto;
import frc.robot.commands.autos.CenterOnceBumpDepotOrOutpostAuto;
import frc.robot.commands.autos.CenterOnceTrenchAuto;
import frc.robot.commands.autos.CenterTwiceBumpAuto;
import frc.robot.commands.autos.CenterTwiceBumpAutoWithBriefDelay;
import frc.robot.commands.autos.CenterTwiceBumpAutoWithShootDelay;
import frc.robot.commands.autos.CenterTwiceTrenchAuto;
import frc.robot.commands.autos.CenterTwiceTrenchAutoWithBriefDelay;
import frc.robot.commands.autos.CenterTwiceTrenchAutoWithShootDelay;
import frc.robot.commands.autos.CustomAuto;
import frc.robot.commands.autos.DepotAuto;
import frc.robot.commands.autos.OutpostAndDepotAuto;
import frc.robot.commands.autos.OutpostAuto;
import frc.robot.commands.autos.ShootAuto;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path.PathConstraints;

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

    // Every path the autos use, loaded at startup so building an auto doesn't have to read files
    private static final String[] autoPathNames = {
            "Trench-to-Center",
            "Center-to-Trench",
            "Trench-to-Leftovers",
            "Leftovers-to-Trench",
            "Center-to-Bump",
            "Bump-to-Leftovers",
            "Center-to-Leftovers",
            "Bump-to-Outpost",
            "Bump-to-Depot",
            "Trench-to-Outpost",
            "Outpost-to-Depot",
            "Depot-to-Outpost",
            "Shoot-to-Trench",
            "Trench-to-Collect",
    };

    public Timer timer = new Timer();

    public AutoCommands(RobotContainer robot) {
        for (String pathName : autoPathNames) {
            AutoPaths.getPath(pathName);
        }
        this.robot = robot;

        m_chooser.addOption("Outpost Auto", "Outpost Auto");
        m_chooser.addOption("Depot Auto", "Depot Auto");
        m_chooser.addOption("Outpost and Depot Auto", "Outpost and Depot Auto");
        m_chooser.addOption("Shoot Auto", "Shoot Auto");
        m_chooser.addOption("Center Twice Trench Auto", "Center Twice Trench Auto");
        m_chooser.addOption("Center Twice Bump Auto", "Center Twice Bump Auto");
        m_chooser.addOption("Custom", "Custom");
        m_chooser.addOption("Mechanical Advantage Delay", "Mechanical Advantage Delay");
        m_chooser.setDefaultOption("none", "none");
        SmartDashboard.putData("autoChooser", m_chooser);

        SmartDashboard.putData("setPoseToLeftTrench", resetRobotPose(leftTrenchPose));
        SmartDashboard.putData("setPoseToRightTrench", resetRobotPose(rightTrenchPose));
        SmartDashboard.putData("setPoseToHub", resetRobotPose(hubPose));

        NTHelper.setStringArray("/SmartDashboard/someStringArray", new String[] { "Default" });
        NTHelper.setDoubleArray("/SmartDashboard/setSomeNumberArray", new double[] { 0.0 });

        NTHelper.listen("/SmartDashboard/someStringArray", (value) -> {
             NTHelper.setDoubleArray("/SmartDashboard/autoPathPoints", CustomAuto.getCombinedPath(
                    NTHelper.getStringArray("/SmartDashboard/someStringArray", new String[] { "Default" })));
        });

    }

    private Command resetRobotPose(Pose2d pose) {
        Command command = Commands.runOnce(() -> {
            robot.drivetrain.resetPose(PoseTransformUtils.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose);
        });
        command.runsWhenDisabled();
        return command;
    }

    private Command customAuto(String[] steps, double[] times) {
        return new CustomAuto(robot, timer, steps, times).build();
    }

    public Command getAuto() {
        if (m_chooser.getSelected().equals("Center Once Trench Auto")) {
            return new CenterOnceTrenchAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Outpost Auto")) {
            return new OutpostAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Outpost and Depot Auto")) {
            return new OutpostAndDepotAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Depot Auto")) {
            return new DepotAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Shoot Auto")) {
            return new ShootAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto")) {
            return new CenterTwiceTrenchAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Breif Delay")) {
            return new CenterTwiceTrenchAutoWithBriefDelay(robot, 1.5).build();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Shoot Delay")) {
            return new CenterTwiceTrenchAutoWithShootDelay(robot, 3).build();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Long Shoot Delay")) {
            return new CenterTwiceTrenchAutoWithShootDelay(robot, 4).build();
        } else if (m_chooser.getSelected().equals("Center Twice Trench Auto with Outpost or Depot Delay")) {
            return new CenterTwiceTrenchAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto")) {
            return new CenterTwiceBumpAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Custom Center Twice Bump Auto")) {
            return customAuto(new String[] { "", "bump", "bump" }, new double[] { 0, 10 });
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Breif Delay")) {
            return new CenterTwiceBumpAutoWithBriefDelay(robot, 1.5).build();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Shoot Delay")) {
            return new CenterTwiceBumpAutoWithShootDelay(robot, 3).build();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Long Shoot Delay")) {
            return new CenterTwiceBumpAutoWithShootDelay(robot, 4).build();
        } else if (m_chooser.getSelected().equals("Center Twice Bump Auto with Outpost or Depot Delay")) {
            return new CenterTwiceBumpAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot or Outpost Auto")) {
            return new CenterOnceBumpDepotOrOutpostAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Center Once Bump Depot and Outpost Auto")) {
            return new CenterOnceBumpDepotAndOutpostAuto(robot).build();
        } else if (m_chooser.getSelected().equals("Custom")) {

            return customAuto(
                    NTHelper.getStringArray("/SmartDashboard/someStringArray", new String[] { "Default" }),
                    NTHelper.getDoubleArray("/SmartDashboard/setSomeNumberArray", new double[] { 0.0 }));
        } else if (m_chooser.getSelected().equals("Center Once Bump Collect Auto")) {
            return customAuto(new String[] { "", "bump", "collect" }, new double[] { 0, 12 });
        } else if (m_chooser.getSelected().equals("Mechanical Advantage Delay")) {
            return customAuto(new String[] { "shoot", "bump", "bump" }, new double[] { 4.5, 14 });
        } else if (m_chooser.getSelected().equals("Custom Center Once Bump Collect Auto with Outpost or Depot Delay")) {
            return customAuto(new String[] { "outpost or depot", "trench", "collect" }, new double[] { 5, 14 });
        } else if (m_chooser.getSelected().equals("Test")) {
            return customAuto(new String[] { "none", "trench", "bump" }, new double[] { 0, 14 });
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
