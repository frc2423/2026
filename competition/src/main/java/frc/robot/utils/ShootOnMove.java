package frc.robot.utils;

import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ShootOnMove {

    private static float projectileTime = 2;
    private CommandSwerveDrivetrain drivetrain;
    private Pose2d hubPose = new Pose2d();

    public ShootOnMove(CommandSwerveDrivetrain initialDrivetrain) {
        drivetrain = initialDrivetrain;
    }

    public Pose2d getMovingTargetPose2d() {
        Pose2d targetPose = new Pose2d(
                hubPose.getX() - (drivetrain.getState().Speeds.vxMetersPerSecond * projectileTime),
                hubPose.getY() - (drivetrain.getState().Speeds.vyMetersPerSecond * projectileTime), new Rotation2d());
        return targetPose;
    }
}
