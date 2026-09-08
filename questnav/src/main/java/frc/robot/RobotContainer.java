// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.Vision;
import frc.robot.telemetry.Telemetry;


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

        private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(12);
        private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(12);

        public final CommandXboxController driverController = new CommandXboxController(0);
        public final CommandXboxController operatorController = new CommandXboxController(1);
        public final CommandXboxController shooterTuningController = new CommandXboxController(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(10, 0, 0);
        private Rotation2d lastHeading = new Rotation2d();

        private Supplier<Pose2d> currentPose;

        // @Logged
        public final Vision vision = new Vision(currentPose);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final PowerDistribution pdh = new PowerDistribution();

        public RobotContainer() {
                SmartDashboard.putData("pdh", pdh);

                configureBindings();
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

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureDriveControllerBindings() {
                // reset the field-centric heading on left bumper press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        }


        private void configureOperatorControllerBindings() {

                // operatorController.leftBumper().whileTrue(Commands.waitSeconds(.5).andThen(shooterCommands.feed(()
                // -> {
                // return NTHelper.getDouble("/tuning/FeederSpeed", 1);
                // })))
                // .onFalse(shooterCommands.stopFeeding());

                // operatorController.rightBumper()
                // .whileTrue(shooterCommands.rev(() ->
                // NTHelper.getDouble("/tuning/ShooterSpeed", 0)))
                // .onFalse(shooterCommands.stopShooting());

                operatorController.button(7).onTrue(new InstantCommand(() -> drivetrain
                                .resetRotation(new Rotation2d(PoseTransformUtils.isRedAlliance() ? 180 : 0))));
                operatorController.button(8).onTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d())));
        }

}