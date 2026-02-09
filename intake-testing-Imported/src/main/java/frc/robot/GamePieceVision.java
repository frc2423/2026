package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GamePieceVision {
  private final PhotonCamera camera;
  private final Transform2d robotToCamera; // robot->camera transform in robot frame

  // Measure these on your robot:
  private final double cameraHeightMeters;
  private final double cameraPitchRad; // +pitch up

  public GamePieceVision(
      PhotonCamera camera,
      Transform2d robotToCamera,
      double cameraHeightMeters,
      double cameraPitchRad) {
    this.camera = camera;
    this.robotToCamera = robotToCamera;
    this.cameraHeightMeters = cameraHeightMeters;
    this.cameraPitchRad = cameraPitchRad;
  }

  /** Returns null if we can't estimate a sane point. */
  public Translation2d estimateFieldPiecePoint(Pose2d fieldToRobot, PhotonTrackedTarget t) {
    // For a floor object, targetHeight ~ 0 (meters). If your piece sits higher, use that.
    double targetHeightMeters = 0.0;

    double targetPitchRad = Units.degreesToRadians(t.getPitch());
    double targetYawRad   = Units.degreesToRadians(t.getYaw());

    // Range estimate from pitch (ray-floor intersection style)
    double rangeMeters = PhotonUtils.calculateDistanceToTargetMeters(
        cameraHeightMeters, targetHeightMeters, cameraPitchRad, targetPitchRad);

    if (!Double.isFinite(rangeMeters) || rangeMeters < 0.2 || rangeMeters > 6.0) {
      return null; // reject nonsense
    }

    // Camera-relative translation (x forward, y left in WPILib/Photon conventions)
Translation2d cameraToTarget = new Translation2d(
    rangeMeters * Math.cos(targetYawRad), // forward
    rangeMeters * Math.sin(targetYawRad)  // left
);

    // field->camera pose
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera);

    // apply camera->target in camera frame
    Transform2d cameraToTargetTf = new Transform2d(cameraToTarget, new Rotation2d());
    Pose2d fieldToTarget = fieldToCamera.transformBy(cameraToTargetTf);

    return fieldToTarget.getTranslation();
  }
}
