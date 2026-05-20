package frc.robot.subsystems;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.QuestNavConstants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation3d;

public class QuestNavSubsystem extends SubsystemBase {

    public QuestNavSubsystem() {

    }

    QuestNav questNav = new QuestNav();
    CommandSwerveDrivetrain swerveDrivetrain = TunerConstants.createDrivetrain();
    QuestNavConstants questConstants = new QuestNavConstants();
     
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02,        // X position trust (20 mm)
            0.02,        // Y position trust (20 mm)
            0.0872665);  // Rotation trust (5 degrees)

    @Override
    public void periodic() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(questConstants.ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                swerveDrivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }
    }

}