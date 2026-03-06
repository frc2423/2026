// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.PassingCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BLine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveShortestPath;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TwindexerSubsystem;
import frc.robot.subsystems.LEDS.KwarqsLed;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShootOnMove;
import frc.robot.telemetry.SubsystemMechanism2d;
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

        private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(7);
        private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(7);

        public final CommandXboxController driverController = new CommandXboxController(0);
        public final CommandXboxController operatorController = new CommandXboxController(1);
        public final IntakeSubsystem intake = new IntakeSubsystem();

        @Logged
        public final ArmSubsystem arm = new ArmSubsystem();
        @Logged
        public final HoodSubsystem hood = new HoodSubsystem();
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

        public final BLine bline = new BLine(drivetrain);

        @Logged
        public KwarqsLed kwarqsLed = new KwarqsLed();

        @Logged
        public final ShooterCommands shooterCommands = new ShooterCommands(this);

        public final PassingCommands passingCommands = new PassingCommands(this);
        public final IntakeCommands intakeCommands = new IntakeCommands(this);
        public final ShootOnMove shootOnMove = new ShootOnMove(drivetrain);
        public final DriveShortestPath driveShortestPath = new DriveShortestPath(drivetrain, bline);
        public final AutoCommands auto = new AutoCommands(this);

        private final Telemetry logger = new Telemetry(MaxSpeed);
        @SuppressWarnings("unused")
        private final SubsystemMechanism2d subsystemMechanism2d = new SubsystemMechanism2d(this);

        public RobotContainer() {
                SmartDashboard.putData("subsystems/arm", arm);
                SmartDashboard.putData("subsystems/feederLeft", feederLeft);
                SmartDashboard.putData("subsystems/feederRight", feederRight);
                SmartDashboard.putData("subsystems/intake", intake);
                SmartDashboard.putData("subsystems/shooterLeft", shooterLeft);
                SmartDashboard.putData("subsystems/shooterRight", shooterRight);
                SmartDashboard.putData("subsystems/twindexer", twindexer);
                SmartDashboard.putData("subsytems/hood", hood);

                configureBindings();
                NTHelper.setDouble("/tuning/FeederSpeed", 1);
                NTHelper.setDouble("/tuning/ShooterSpeed", 2800);
                NTHelper.setBoolean("/tuning/snakeMode", false);

        }

        private void configureLeds() {
                kwarqsLed.setDefaultCommand(kwarqsLed.setLeds(() -> {
                        if (drivetrain.isSeeingAprilTag()) {
                                return PoseTransformUtils.isRedAlliance() ? "RedCycle" : "BlueCycle";
                        }
                        if (RobotState.isAutonomous()) {
                                return "rainbow";
                        }
                        return "dark";
                }));
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> {

                                        shooterCommands.isFacingHub();
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
                configureLeds();

                drivetrain.registerTelemetry(logger::telemeterize);

                RobotModeTriggers.disabled().onTrue(disableEverything());

        }

        private void configureDriveControllerBindings() {
                // reset the field-centric heading on left bumper press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Intake commands
                driverController.button(9).whileTrue(intake.outtake()).onFalse(intake.stop());
                driverController.button(10).whileTrue(intakeCommands.armDown().andThen(intake.intake()))
                                .onFalse(intake.stop());
                driverController.b().onTrue(arm.armUp());
                driverController.a().onTrue(intakeCommands.armDown());
                // driverController.x().onTrue(hood.set(.1)).onFalse(hood.set(0));
                // driverController.y().onTrue(hood.set(-.1)).onFalse(hood.set(0));

                driverController.rightTrigger(0.25).whileTrue(shooterCommands.prepareToShoot())
                                .onFalse(shooterCommands.stopShooting());

                driverController.rightBumper()
                                .whileTrue(shooterCommands.feed(() -> NTHelper.getDouble("/tuning/FeederSpeed", 0)))
                                .onFalse(shooterCommands.stopFeeding());
                driverController.leftBumper().whileTrue(passingCommands.trenchPass()).onFalse(intake.stop());
                driverController.leftTrigger().whileTrue(passingCommands.aimToPass())
                                .onFalse(shooterCommands.stopShooting());

        }

        private void configureOperatorControllerBindings() {

                operatorController.leftBumper().whileTrue(Commands.waitSeconds(.5).andThen(shooterCommands.feed(() -> {
                        return NTHelper.getDouble("/tuning/FeederSpeed", 1);
                })))
                                .onFalse(shooterCommands.stopFeeding());

                operatorController.rightBumper()
                                .whileTrue(shooterCommands.rev(() -> NTHelper.getDouble("/tuning/ShooterSpeed", 0)))
                                .onFalse(shooterCommands.stopShooting());

                operatorController.a().whileTrue(intakeCommands.armDown());
                operatorController.b().whileTrue(arm.armUp());
                operatorController.x().whileTrue(intake.intake()).onFalse(intake.stop());
                operatorController.y().whileTrue(intake.outtake()).onFalse(intake.stop());

                operatorController.povLeft().whileTrue(twindexer.spindexBack()).onFalse(twindexer.stop());
                operatorController.povRight().whileTrue(twindexer.spindex()).onFalse(twindexer.stop());

                // operatorController.povUp().whileTrue(feederLeft.spinWithSetpoint(() ->
                // 0.7).alongWith(feederRight.spinWithSetpoint(() ->
                // 0.7))).onFalse(feederLeft.stop().alongWith(feederRight.stop()));
                // operatorController.povDown().whileTrue(feederLeft.spinWithSetpoint(() ->
                // -0.7).alongWith(feederRight.spinWithSetpoint(() ->
                // -0.7))).onFalse(feederLeft.stop().alongWith(feederRight.stop()));

                // operatorController.leftBumper().whileTrue(passingCommands.trenchPass());
                // operatorController.leftTrigger().whileTrue(passingCommands.aimToPass());

        }

        private final CommandXboxController shortestPathController = new CommandXboxController(2);

        private void configureShortestPathBindings() {
                shortestPathController.a().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(14.6, 1.6, new Rotation2d(Math.PI))));
                shortestPathController.b().whileTrue(
                                driveShortestPath.driveShortestPath(new Pose2d(9, 2, new Rotation2d(Math.PI))));
        }

        public Command getAutonomousCommand() {
                return auto.getAuto();
        }

        public Command disableEverything() {
                Command command = Commands.parallel(
                                arm.set(0),
                                feederLeft.stop(),
                                feederRight.stop(),
                                hood.set(0),
                                intake.stop(),
                                shooterLeft.stop(),
                                shooterRight.stop(),
                                twindexer.stop()).ignoringDisable(true);
                return command;
        }

}