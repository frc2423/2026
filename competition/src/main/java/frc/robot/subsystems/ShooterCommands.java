package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FlippingUtil;

public class ShooterCommands extends SubsystemBase {

  private ShooterSubsystem shooterR;
  private ShooterSubsystem shooterL;
  private FeederSubsystem feederR;
  private FeederSubsystem feederL;
  private TwindexerSubsystem twinDexer;

  private CommandSwerveDrivetrain swerve;
  public static final DAS das = new DAS();
  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withHeadingPID(10, 0, 0);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

  public ShooterCommands(ShooterSubsystem shooterR, ShooterSubsystem shooterL, FeederSubsystem feederR,
      FeederSubsystem feederL, TwindexerSubsystem twinDexer, CommandSwerveDrivetrain swerve) {
    this.shooterR = shooterR;
    this.shooterL = shooterL;
    this.swerve = swerve;
    this.feederR = feederR;
    this.feederL = feederL;
    this.twinDexer = twinDexer;

  }

  public double getDistanceBetweenPoses(Pose2d a, Pose2d b) {
    double y = a.getY() - b.getY();
    double x = a.getX() - b.getX();
    return Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
  }

  @Logged
  public double getDistanceToHub() {
    // Pose2d = Pose2d.kZero.getTranslation().getDistance()
    return getDistanceBetweenPoses(swerve.getPose(),
        new Pose2d((PoseTransformUtils.isRedAlliance())
            ? FlippingUtil.flipFieldPosition(FieldConstants.Hub.topCenterPoint.toTranslation2d())
            : FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));

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

  public Command acuallyLookAngle() {
    return swerve.applyRequest(() -> {
      double x = xSpeedLimiter.calculate(driverController.getLeftY() * MaxSpeed);
      double y = ySpeedLimiter.calculate(driverController.getLeftX() * MaxSpeed);
      Rotation2d targetHeading = getLookAngle(
          new Pose2d((PoseTransformUtils.isRedAlliance())
              ? FlippingUtil.flipFieldPosition(FieldConstants.Hub.topCenterPoint.toTranslation2d())
              : FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero));

      return driveFacing
          .withVelocityX(-x)
          .withVelocityY(-y)
          .withTargetDirection(targetHeading);
    });
  }

  public Command prepareToShoot() {
    Command command = Commands.parallel(acuallyLookAngle(), revSpeedFromDAS());
    return command;
  }

  // TODO: Make this use feeders and twindexer, not shooter
  public Command spinFeeder(Supplier<Double> setpoint) {

    Command feedersAndTwindexer = Commands.parallel(
        feederL.spin(() -> setpoint.get()),
        feederR.spin(() -> setpoint.get()),
        Commands.repeatingSequence(twinDexer.spindex().until(() -> twinDexer.isJammed()).andThen(twinDexer.spindexBack()).withTimeout(0.5)));
    // Command command = Commands.parallel(shooterL.spinWithSetpoint(() -> setpoint),
    //     shooterR.spinWithSetpoint(() -> setpoint));

    return feedersAndTwindexer;

  }

  private Command revSpeedFromDAS() {
    Command left = shooterR.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return as.velocity;
    });
    Command right = shooterL.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return as.velocity;
    });
    return Commands.parallel(left, right);
  }

}
