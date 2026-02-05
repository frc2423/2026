// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BLine;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);

    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooterLeft = new ShooterSubsystem(5);
    public final ShooterSubsystem shooterRight = new ShooterSubsystem(7);
    public final BLine bline = new BLine(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double x = xSpeedLimiter.calculate(driverController.getLeftY() * MaxSpeed);
                    double y = ySpeedLimiter.calculate(driverController.getLeftX() * MaxSpeed);
                    return drive.withVelocityX(x) // Drive forward with negative Y (forward)
                            .withVelocityY(y) // Drive left with negative X (left)
                            .withRotationalRate(-driverController.getRightX() * MaxAngularRate); // Drive
                                                                                                 // counterclockwise
                                                                                                 // with
                    // negative X (left)
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driverController.leftBumper().whileTrue(shooterLeft.spin()).onFalse(shooterLeft.stop());
        driverController.rightBumper().whileTrue(shooterRight.spin()).onFalse(shooterRight.stop());
        driverController.leftBumper().and(driverController.rightBumper())
                .whileTrue((shooterRight.spin()).alongWith(shooterLeft.spin()))
                .onFalse(shooterLeft.stop().alongWith(shooterRight.stop()));

        driverController.x().whileTrue(intake.spin()).onFalse(intake.stop());
        driverController.y().whileTrue(intake.outtake()).onFalse(intake.stop());

        driverController.a().whileTrue(bline.goToPose(new Pose2d(1, 1, Rotation2d.kZero)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
