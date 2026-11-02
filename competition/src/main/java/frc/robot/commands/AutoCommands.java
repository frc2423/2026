package frc.robot.commands;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.CenterTwiceBumpAuto;
import frc.robot.commands.autos.CenterTwiceTrenchAuto;
import frc.robot.commands.autos.ChargedUpTestAuto;
import frc.robot.commands.autos.CustomAuto;
import frc.robot.commands.autos.DepotAuto;
import frc.robot.commands.autos.MechanicalAdvantageDelayAuto;
import frc.robot.commands.autos.OutpostAndDepotAuto;
import frc.robot.commands.autos.OutpostAuto;
import frc.robot.commands.autos.ShootAuto;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.lib.BLine.FlippingUtil;

public class AutoCommands {
    private final RobotContainer robot;

    // Autos shown in the chooser, in order. Each class needs a constructor that takes just a RobotContainer.
    private static final Map<String, Class<? extends AutoRoutine>> autos = new LinkedHashMap<>();
    static {
        autos.put("Outpost Auto", OutpostAuto.class);
        autos.put("Depot Auto", DepotAuto.class);
        autos.put("Outpost and Depot Auto", OutpostAndDepotAuto.class);
        autos.put("Shoot Auto", ShootAuto.class);
        autos.put("Center Twice Trench Auto", CenterTwiceTrenchAuto.class);
        autos.put("Center Twice Bump Auto", CenterTwiceBumpAuto.class);
        autos.put("Custom", CustomAuto.class);
        autos.put("Mechanical Advantage Delay", MechanicalAdvantageDelayAuto.class);
        autos.put("Charged Up Test Auto", ChargedUpTestAuto.class);
    }

    // @Logged
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private static final Pose2d leftTrenchPose = new Pose2d(3.5, 7.5, Rotation2d.fromDegrees(180));
    private static final Pose2d rightTrenchPose = new Pose2d(3.5, 0.5, Rotation2d.fromDegrees(180));
    private static final Pose2d hubPose = new Pose2d(3.5, 4, Rotation2d.fromDegrees(180));

    public Timer timer = new Timer();

    public AutoCommands(RobotContainer robot) {
        // Load paths at startup so building an auto doesn't have to read files
        AutoPaths.loadAllPaths();
        AutoPaths.loadAllPaths("chargedup-autos");
        this.robot = robot;

        for (String autoName : autos.keySet()) {
            m_chooser.addOption(autoName, autoName);
        }
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

    public Command getAuto() {
        String selected = m_chooser.getSelected();
        Class<? extends AutoRoutine> autoClass = autos.get(selected);
        if (autoClass == null) {
            return Commands.none();
        }
        try {
            return autoClass.getConstructor(RobotContainer.class).newInstance(robot).build();
        } catch (ReflectiveOperationException e) {
            DriverStation.reportError("Failed to create auto \"" + selected + "\": " + e, e.getStackTrace());
            return Commands.none();
        }
    }
}
