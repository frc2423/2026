package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.NTHelper;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.subsystems.DAS;
import frc.robot.subsystems.Vision;

public class ShooterCommands extends SubsystemBase {

  private final RobotContainer robot;

  public static final DAS das = new DAS();
  private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withHeadingPID(3, 0, 0);

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

  @Logged
  private String selectedShootingPose = "Auto";

  public ShooterCommands(RobotContainer robot) {
    this.robot = robot;
  }

  public double getDistanceBetweenPoses(Pose2d a, Pose2d b) {
    double y = a.getY() - b.getY();
    double x = a.getX() - b.getX();
    return Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
  }

  @Logged
  public double getDistanceToHub() {
    return getDistanceBetweenPoses(getShootingPose(), getHubPose());

  }

  public void setFixedShootingPose(String shootingPose) {
    selectedShootingPose = shootingPose;
  }

  private Pose2d getShootingPose() {
    Pose2d shootingPose = robot.drivetrain.getPose();
    if (selectedShootingPose == null) {
      return shootingPose;
    }
    if (selectedShootingPose.equals("Tower")) {
      shootingPose = new Pose2d(1.72, 3.63, robot.drivetrain.getPose().getRotation());
    } else if (selectedShootingPose.equals("Left Bump")) {
      shootingPose = new Pose2d(3.23, 6.4, robot.drivetrain.getPose().getRotation());
    } else if (selectedShootingPose.equals("Right Bump")) {
      shootingPose = new Pose2d(3.23, 1.65, robot.drivetrain.getPose().getRotation());
    }
    if (!selectedShootingPose.equals("Auto") && PoseTransformUtils.isRedAlliance()) {
      shootingPose = FlippingUtil.flipFieldPose(shootingPose);
    }
    return shootingPose;
  }

  public Rotation2d getLookAngle(Pose2d targetPose) {
    Pose2d currentPose = getShootingPose();
    double distance = getDistanceBetweenPoses(currentPose, targetPose);
    if (distance < Units.inchesToMeters(8)) {
      return currentPose.getRotation();
    }
    double angleRads = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
    if (PoseTransformUtils.isRedAlliance()) {
      angleRads += Math.PI;
    }
    return new Rotation2d(angleRads);
  }

  public Pose2d getHubPose() {
    Translation2d hubTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (PoseTransformUtils.isRedAlliance()) {
      hubTranslation = FlippingUtil.flipFieldPosition(hubTranslation);
    }
    return new Pose2d(hubTranslation, Rotation2d.kZero);
  }

  public Command lookAtAngle(Rotation2d targetHeading) {
    return robot.drivetrain.applyRequest(() -> {
      return driveFacing
          .withTargetDirection(targetHeading);
    }).until(() -> isFacingAngle(targetHeading));
  }

  public Command lookAtAprilTag() {
    return robot.drivetrain.applyRequest(() -> {
      double yaw = robot.vision.getRelativeBinYaw(180);
      yaw = -yaw * .1;
      yaw = MathUtil.clamp(yaw, -.7, .7);
      
      NTHelper.setDouble("/coolyaw/scaledYaw", yaw);
      if (Math.abs(yaw) < .3) {
        yaw = 0;
      }
      return drive
        .withRotationalRate(yaw);
    });
  }

  public Command lookAtPose() {
    return lookAtPose(() -> getHubPose());
  }

  public Command lookAtPose(Pose2d targetPose) {
    return lookAtPose(() -> targetPose);

  }

  public Command lookAtPose(Supplier<Pose2d> targetPoseSupplier) {
    return robot.drivetrain.applyRequest(() -> {

       double x = xSpeedLimiter.calculate(driverController.getLeftY() * MaxSpeed);
      double y = ySpeedLimiter.calculate(driverController.getLeftX() * MaxSpeed);

      x = MathUtil.applyDeadband(x, .25*MaxSpeed);
      y = MathUtil.applyDeadband(y, .25*MaxSpeed);

      // double x = driverController.axisMagnitudeGreaterThan(0, .25).getAsBoolean()?xSpeedLimiter.calculate(driverController.getLeftY() * MaxSpeed):0;
      // double y = driverController.axisMagnitudeGreaterThan(2, .25).getAsBoolean()?ySpeedLimiter.calculate(driverController.getLeftX() * MaxSpeed):0;
      Rotation2d targetHeading = getLookAngle(targetPoseSupplier.get()).plus(Rotation2d.k180deg);

      return driveFacing
          .withVelocityX(-x)
          .withVelocityY(-y)
          .withTargetDirection(targetHeading);
    });
  }

  public boolean isFacingPose(Pose2d targetPose) {
    Angle targetAngle = getLookAngle(targetPose).getMeasure();
    Angle robotAngle = PoseTransformUtils.isRedAlliance() ? getShootingPose().getRotation().getMeasure()
        : getShootingPose().getRotation().getMeasure().plus(Degrees.of(180));
    if (robotAngle.in(Degrees) < 0) {
      robotAngle = Degrees.of(robotAngle.plus(Degrees.of(360)).in(Degrees));
    }
    if (targetAngle.in(Degrees) < 0) {
      targetAngle = Degrees.of(targetAngle.plus(Degrees.of(360)).in(Degrees));
    }
    // System.out.println("targetAngle: " + targetAngle.in(Degrees) + ", robotAngle:
    // " + robotAngle.in(Degrees));
    boolean isNear = targetAngle.isNear(robotAngle, Degrees.of(10));
    return isNear;
  }

  public boolean isFacingAngle(Rotation2d targetAngle) {
    return getShootingPose().getRotation().getMeasure().isNear(targetAngle.getMeasure(), Degrees.of(3));
  }

  public boolean isFacingHub() {
    return isFacingPose(getHubPose());
  }

  public Command prepareToShoot() {
    Command command = Commands.parallel(lookAtAprilTag(), revSpeedFromDAS());
    return command;
  }

  public Command scoreDeadline(double shootingTime) {
    return Commands.sequence(
        prepareToShoot().until(() -> isFacingHub()),
        Commands.parallel(
            lookAtPose(),
            revSpeedFromDAS(),
            feed()).withTimeout(shootingTime));
  }

  public Command scoreUntil(BooleanSupplier stopCondition) {
    return Commands.sequence(
        prepareToShoot().until(() -> isFacingHub()),
        Commands.parallel(
            lookAtPose(),
            revSpeedFromDAS(),
            feed()).until(stopCondition));
  }

  public Command feedOnly() {

    Command spindex = Commands.repeatingSequence(
        robot.twindexer.spindex(),
        Commands.waitUntil(() -> robot.twindexer.isJammed()),
        robot.twindexer.spindexBack(),
        Commands.waitSeconds(.5));

    Command feed = Commands.parallel(
        robot.feederLeft.spin(),
        robot.feederRight.spin(),
        robot.arm.wiggleArm(Degrees.of(110), Degrees.of(35), Seconds.of(.55)),
        robot.intake.intakeSlow(),
        Commands.waitSeconds(.5).andThen(spindex));

    return feed;
  }

  public Command feed() {

    Command waitUntilRevved = Commands.waitUntil(() -> {
      if (Robot.isSimulation()) {
        return true;
      }
      return robot.shooterLeft.isAtSetpoint();
    });

    return Commands.parallel(
        positionHoodFromDas(),
        waitUntilRevved.andThen(feedOnly()));
  }

  public Command stopFeeding() {
    return Commands.sequence(
        robot.feederLeft.stop(),
        robot.feederRight.stop(),
        robot.twindexer.stop(),
        robot.intake.stop());
  }

  public Command stopShooting() {
    return Commands.sequence(
        robot.shooterLeft.stop(),
        robot.shooterRight.stop());
  }

  public Command positionHoodFromDas() {
    return robot.hood.setAngle(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return Degrees.of(25.0);
    });
  }

  public Command revSpeedFromDAS() {
    Command left = robot.shooterLeft.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return 2500.0;
    });
    Command right = robot.shooterRight.spinWithSetpoint(() -> {
      double distance = this.getDistanceToHub(); // not real
      DAS.MotorSettings as = das.calculateAS(distance);
      return 2500.0;
    });

    return Commands.parallel(
        left,
        right
    // positionHoodFromDas()
    );
  }

  public Command rev(Supplier<Double> speed) {
    return Commands.parallel(
        robot.shooterLeft.spinWithSetpoint(speed),
        robot.shooterRight.spinWithSetpoint(speed));
  }

}
