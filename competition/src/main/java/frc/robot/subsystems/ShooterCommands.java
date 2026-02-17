package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;

public class ShooterCommands extends SubsystemBase {

  private ShooterSubsystem motorR;
  private ShooterSubsystem motorL;

  private CommandSwerveDrivetrain swerve;
  public static final DAS das = new DAS();
  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withHeadingPID(10, 0, 0);

  public ShooterCommands(ShooterSubsystem motorR, ShooterSubsystem motorL, CommandSwerveDrivetrain swerve) {
    this.motorR = motorR;
    this.motorL = motorL;
    this.swerve = swerve;

  }

  public double getDistanceBetweenPoses(Pose2d a, Pose2d b) {
    double y = a.getY() - b.getY();
    double x = a.getX() - b.getX();
    return Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
  }

  public double getDistanceToHub() {
    // Pose2d = Pose2d.kZero.getTranslation().getDistance()
    return getDistanceBetweenPoses(swerve.getPose(),
        new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));

  }

  // TODO: Works for the blue alliance but not red
  public Rotation2d getLookAngle(Pose2d targetPose) {
    Pose2d currentPose = swerve.getPose();
    double distance = getDistanceBetweenPoses(currentPose, targetPose);
    if (distance < Units.inchesToMeters(8)) {
      return currentPose.getRotation();
    }
    double angleRads = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
    return new Rotation2d(angleRads);
  }

  // TODO: Make x and y translation use the same code as the teleop drive command in RobotContainer 
  public Command acuallyLookAngle() {
    return swerve.applyRequest(() -> {
      double x = 0;
      double y = 0;
      Rotation2d targetHeading = getLookAngle(
          new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));

      return driveFacing
          .withVelocityX(x)
          .withVelocityY(y)
          .withTargetDirection(targetHeading);
    });
  }

  public Command prepareToShoot() {
    Command command = Commands.parallel(acuallyLookAngle(), revSpeedFromDAS());
    return command;
  }

  // TODO: Make this use feeders and twindexer, not shooter
  public Command spinFeeder(double setpoint) {

    // Command feedersAndTwindexer = Commands.parallel(
    //     feederLeft.spin(setpoint),
    //     feederRight.spin(setpoint),
    //     twindexer.spindex());
    Command command = Commands.parallel(motorL.spinWithSetpoint(() -> setpoint),
    motorR.spinWithSetpoint(() -> setpoint));

    return command;

  }

  private Command revSpeedFromDAS() {
    return Commands.run(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      // motor.setPidSpeed(as.getVelocity());
      motorR.spinWithSetpoint(() -> as.velocity);
      motorL.spinWithSetpoint(() -> as.velocity);

    });

  }

}
