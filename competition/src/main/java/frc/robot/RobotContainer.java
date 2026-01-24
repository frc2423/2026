// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double x = xSpeedLimiter.calculate(-driver.getLeftY() * MaxSpeed);
                    double y = ySpeedLimiter.calculate(-driver.getLeftX() * MaxSpeed);
                    return drive.withVelocityX(x) // Drive forward with negative Y (forward)
                            .withVelocityY(y) // Drive left with negative X (left)
                            .withRotationalRate(-driver.getRightX() * MaxAngularRate); // Drive counterclockwise with
                                                                                       // negative X (left)
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        // // point.withModuleDirection(new Rotation2d(-driver.getLeftY(),
        // -driver.getLeftX()))
        // point.withModuleDirection(new Rotation2d(0))
        // ));

        driver.a().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(xSpeedLimiter.calculate(-.5)).withDeadband(0)));
        driver.b().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(xSpeedLimiter.calculate(.5)).withDeadband(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        driver.button(10).whileTrue(intake.spin()).onFalse(intake.stop());
        driver.button(9).whileTrue(intake.outtake()).onFalse(intake.stop());
        // driver.x().whileTrue(intake.intakeIn()).onFalse(intake.intakeStop());
        // driver.y().whileTrue(intake.intakeOut()).onFalse(intake.intakeStop());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
