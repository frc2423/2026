// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.Optional;

import swervelib.SwerveInputStream;
import frc.robot.subsystems.QuackNav;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
        // Replace with CommandPS4Controller or CommandJoystick if needed
        // final CommandXboxController driverXbox = new CommandXboxController(0);
        public final XboxController driverXbox = new XboxController(0);
        public final XboxController operator = new XboxController(1);
        String deployDirectory = (Robot.isSimulation()) ? "sim-swerve/neo" : "swerve";
        // The robot's subsystems and commands are defined here...
        public final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), deployDirectory));

        SendableChooser<String> m_chooser = new SendableChooser<>();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

        SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) * Math.PI)
                                                        *
                                                        (Math.PI * 2))
                        .headingWhile(true);

        Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

        Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureDriverBindings();
                Command driveFieldOrientedAngularVelocity = getTeleopDriveCommand();
                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
                DriverStation.silenceJoystickConnectionWarning(true);
                SmartDashboard.putData("autoChooser", m_chooser);
                SmartDashboard.putData("swerveSubsystem", drivebase);
        }

        private Command getTeleopDriveCommand() {
                Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
                                () -> {
                                        double y = MathUtil.applyDeadband(
                                                        driverXbox.getLeftY(),
                                                        OperatorConstants.LEFT_Y_DEADBAND);
                                        if (!PoseTransformUtils.isRedAlliance()) {
                                                y *= -1;
                                        }
                                        return drivebase.m_yspeedLimiter.calculate(y);
                                },
                                () -> {
                                        double x = MathUtil.applyDeadband(
                                                        driverXbox.getLeftX(),
                                                        OperatorConstants.LEFT_X_DEADBAND);
                                        if (!PoseTransformUtils.isRedAlliance()) {
                                                x *= -1;
                                        }
                                        return drivebase.m_xspeedLimiter.calculate(x);
                                },
                                () -> -driverXbox.getRightX());
                return driveFieldOrientedAngularVelocity; // :P
        }

        private void configureDriverBindings() { // RIP isPanel day 0, 2025 -> 3/25/2025

                new JoystickButton(driverXbox, XboxController.Button.kStart.value)
                                .onTrue((new InstantCommand(drivebase::zeroGyro)));

        }

        public Command getAutonomousCommand() {
                return Commands.none();
        }

        public void configureBindings() {
                configureDriverBindings();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

}
