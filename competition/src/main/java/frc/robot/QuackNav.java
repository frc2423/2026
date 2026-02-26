package frc.robot;

import java.util.OptionalDouble;
import java.util.OptionalInt;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuackNav {
    private final QuestNav questNav = new QuestNav();

    /**
     * True once we've called setPose at least once (field reference established).
     */
    private boolean hasInitialPose = false;

    /**
     * Transform from Quest -> Robot (i.e. robotPose =
     * questPose.transformBy(QUEST_TO_ROBOT))
     */
    public static final Transform2d QUEST_TO_ROBOT = new Transform2d(
            Units.inchesToMeters(-2.591),
            Units.inchesToMeters(-8.013),
            Rotation2d.k180deg);

    // Cached latest measurement (robot pose on the field)
    private Pose2d latestRobotPose = Pose2d.kZero;
    private double latestDataTimestampSec = 0.0;
    private boolean latestTracking = false;

    public QuackNav() {
        SmartDashboard.putData("/QuackNavCommands/zeroAngle", zeroAngle());
        SmartDashboard.putData("/QuackNavCommands/zeroPose", zeroPose());
        SmartDashboard.putData("/QuackNavCommands/randomizePose", randomizePose());
        SmartDashboard.putData("/QuackNavCommands/clearInitialPose",
                Commands.runOnce(this::clearInitialPose));
    }

    public void update(CommandSwerveDrivetrain swerveDrive) {
        // Required: process command responses & errors
        questNav.commandPeriodic();

        PoseFrame[] newFrames = questNav.getAllUnreadPoseFrames();
        for (PoseFrame frame : newFrames) {
            if (isQuestMode()) {
                Pose2d questPose2d = frame.questPose3d().toPose2d();

                // Add vision measurement to pose estimator
                swerveDrive.addVisionMeasurement(
                        questPose2d, // Measured pose
                        frame.dataTimestamp(), // When measurement was taken
                        VecBuilder.fill(0.1, 0.1, 0.05) // Standard deviations (tune these)
                );
            }
        }

        // Optional: publish some debug values
        SmartDashboard.putBoolean("/QuackNav/connected", questNav.isConnected());
        SmartDashboard.putBoolean("/QuackNav/tracking", questNav.isTracking());
        SmartDashboard.putBoolean("/QuackNav/hasInitialPose", hasInitialPose);
        SmartDashboard.putNumber("/QuackNav/latencyMs", questNav.getLatency());

        OptionalInt batt = questNav.getBatteryPercent();
        SmartDashboard.putNumber("/QuackNav/batteryPercent", batt.isPresent() ? batt.getAsInt() : -1);

        SmartDashboard.putNumber("/QuackNav/latestDataTimestampSec", latestDataTimestampSec);
        SmartDashboard.putNumber("/QuackNav/x", latestRobotPose.getX());
        SmartDashboard.putNumber("/QuackNav/y", latestRobotPose.getY());
        SmartDashboard.putNumber("/QuackNav/deg", latestRobotPose.getRotation().getDegrees());
    }

    /** “Ready to trust Quest pose frames for field-relative robot pose.” */
    public boolean isQuestMode() {
        return questNav.isConnected() && hasInitialPose && questNav.isTracking();
    }

    public boolean isConnected() {
        return questNav.isConnected();
    }

    public boolean isTracking() {
        return questNav.isTracking();
    }

    public OptionalDouble getAppTimestamp() {
        return questNav.getAppTimestamp();
    }

    /** Latest robot pose derived from Quest frames (cached in periodic). */
    public Pose2d getPose() {
        return latestRobotPose;
    }

    public double getLatestDataTimestampSec() {
        return latestDataTimestampSec;
    }

    public boolean hasInitialPose() {
        return hasInitialPose;
    }

    public void clearInitialPose() {
        hasInitialPose = false;
    }

    public void setRobotPose(Pose2d robotPose) {
        Pose2d questPose2d = robotPose.transformBy(QUEST_TO_ROBOT.inverse());
        questNav.setPose(new Pose3d(questPose2d));
        hasInitialPose = true;
    }


    private Command zeroAngle() {
        return Commands.runOnce(() -> {
            // keep translation, random tiny yaw to "bump" angle if you want
            Pose2d robotPose = new Pose2d(getPose().getTranslation(),
                    Rotation2d.fromDegrees(Math.random() * 0.001));
            setRobotPose(robotPose);
        });
    }

    private Command zeroPose() {
        return Commands.runOnce(() -> {
            setRobotPose(new Pose2d(Math.random() * 0.001, Math.random() * 0.001, Rotation2d.kZero));
        });
    }

    private Command randomizePose() {
        return Commands.runOnce(() -> {
            System.out.println("RANDOMIZE POSE");
            setRobotPose(new Pose2d(
                    Math.random() * 10.0,
                    Math.random() * 5.0,
                    Rotation2d.fromDegrees(Math.random() * 360.0)));
        });
    }
}