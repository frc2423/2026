// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BLine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveShortestPath;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCommands;
import frc.robot.subsystems.TwindexerSubsystem;
import frc.robot.utils.ShootOnMove;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10%
                                                                                                     // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        // private final SwerveRequest.SwerveDriveBrake brake = new
        // SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new
        // SwerveRequest.PointWheelsAt();

        private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
        private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

        public final double feederSpeed = 10;

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driverController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);

        public final IntakeSubsystem intake = new IntakeSubsystem();

        @Logged
        public final ArmSubsystem arm = new ArmSubsystem();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(10, 0, 0);
        private Rotation2d lastHeading = new Rotation2d();
        @Logged
        public final ShooterSubsystem shooterLeft = new ShooterSubsystem(35, true);
        @Logged
        public final ShooterSubsystem shooterRight = new ShooterSubsystem(37, false);
        @Logged
        public final FeederSubsystem feederLeft = new FeederSubsystem(34, false);
        @Logged
        public final FeederSubsystem feederRight = new FeederSubsystem(36, true);
        @Logged
        public final TwindexerSubsystem twindexer = new TwindexerSubsystem();

        @Logged
        public final ShooterCommands shooter = new ShooterCommands(shooterRight, shooterLeft, feederLeft, feederRight, twindexer, drivetrain);

        public final BLine bline = new BLine(drivetrain);
        public final ShootOnMove shootOnMove = new ShootOnMove(drivetrain);
        public final DriveShortestPath driveShortestPath = new DriveShortestPath(drivetrain, bline);

        public RobotContainer() {
                configureBindings();
                SmartDashboard.putData("armSubsystem", arm);
                NTHelper.setDouble("/tuning/FeederSpeed", 1);
                NTHelper.setDouble("/tuning/ShooterSpeed", 2800);
                NTHelper.setBoolean("/tuning/snakeMode", false);

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> {
                                        double x = xSpeedLimiter.calculate(driverController.getLeftY() * MaxSpeed);
                                        double y = ySpeedLimiter.calculate(driverController.getLeftX() * MaxSpeed);

                                        double lx = driverController.getLeftX();
                                        double ly = driverController.getLeftY();
                                        double mag = Math.hypot(lx, ly);

                                        Rotation2d targetHeading;
                                        if (mag > 0.15) {
                                                targetHeading = new Rotation2d(Math.atan2(lx, ly));
                                                lastHeading = targetHeading;
                                        } else {
                                                targetHeading = lastHeading;
                                        }

                                        boolean snakeMode = NTHelper.getBoolean("/tuning/snakeMode", false);

                                        if (snakeMode) {
                                                return driveFacing
                                                                .withVelocityX(-x)
                                                                .withVelocityY(-y)
                                                                .withTargetDirection(
                                                                                targetHeading.plus(Rotation2d.k180deg));
                                        }

                                        return drive.withVelocityX(-x)
                                                        .withVelocityY(-y)
                                                        .withRotationalRate(
                                                                        -driverController.getRightX() * MaxAngularRate);
                                }));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                configureDriveControllerBindings();
                configureOperatorControllerBindings();
                configureShortestPathBindings();

                drivetrain.registerTelemetry(logger::telemeterize);

        }

        private void configureDriveControllerBindings() {
                // reset the field-centric heading on left bumper press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Intake commands
                driverController.x().whileTrue(intake.intake()).onFalse(intake.stop());
                driverController.y().whileTrue(intake.outtake()).onFalse(intake.stop());
                driverController.b().whileTrue(arm.setAngle(Degrees.of(90)));
                driverController.a().whileTrue(arm.setAngle(Degrees.of(10)));

                // Shooting and passing commands
                driverController.rightTrigger(0.25).whileTrue(shooter.prepareToShoot());
                driverController.rightBumper().whileTrue(shooter.spinFeeder(() -> {
                    return NTHelper.getDouble("/tuning/FeederSpeed", 0);    
                }));
                driverController.leftTrigger(0.25)
                                .whileTrue(feederLeft.spin(() -> 0.5).alongWith(feederRight.spin(() -> 0.5)));
                driverController.leftBumper().whileTrue(
                                shooterLeft.spinWithSetpoint(() -> -200.0)
                                                .alongWith(shooterRight.spinWithSetpoint(() -> 200.0)));
        }

        private void configureOperatorControllerBindings() {

                Command feedersAndTwindexer = Commands.parallel(
                                feederLeft.spin(() -> NTHelper.getDouble("/tuning/FeederSpeed", 0)),
                                feederRight.spin(() -> NTHelper.getDouble("/tuning/FeederSpeed", 0)),
                                twindexer.spindex());

                operatorController.leftBumper().whileTrue(Commands.waitSeconds(.5).andThen(feedersAndTwindexer));

                operatorController.rightBumper().whileTrue(Commands.parallel(
                                shooterLeft.spinWithSetpoint(() -> NTHelper.getDouble("/tuning/ShooterSpeed", 0)),
                                shooterRight.spinWithSetpoint(() -> NTHelper.getDouble("/tuning/ShooterSpeed", 0))));

                operatorController.a().whileTrue(twindexer.spindexBack());

        }

        private final CommandXboxController shortestPathController = new CommandXboxController(2);

        // TODO: This don't seem to be working correctly
        private void configureShortestPathBindings() {
                shortestPathController.a().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(6.2, 2, new Rotation2d(Math.PI))));
                shortestPathController.b().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(13, 1.5, new Rotation2d(Math.PI))));
                shortestPathController.x().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(2, 6.5, new Rotation2d(Math.PI))));
                shortestPathController.y().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(8, 7, new Rotation2d(Math.PI))));

                // driverController.a().whileTrue(bline.goToPose(new Pose2d(1, 1,
                // Rotation2d.kZero)));
        }

        private void configureSysIdBindings() {
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driverController.back().and(driverController.y())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driverController.back().and(driverController.x())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driverController.start().and(driverController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driverController.start().and(driverController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        }

        // TODO: Add autos
        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}