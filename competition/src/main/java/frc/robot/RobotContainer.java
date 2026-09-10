// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.PassingCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.PoseTransformUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BLine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveShortestPath;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TwindexerSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.LEDS.KwarqsLed;
import frc.robot.telemetry.DashboardTelemetry;
import frc.robot.telemetry.RobotHealth;
import frc.robot.telemetry.SubsystemMechanism2d;
import frc.robot.telemetry.Telemetry;
import frc.robot.utils.ShootOnMove;

public class RobotContainer {
        
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
        private Orchestra mOrchestra = new Orchestra();

        private SendableChooser<String> song_Chooser = new SendableChooser<String>();
 
        public void setupMusic() {    
                Command startMusic = Commands.runOnce(() -> { // maybe remove track number from network tables, and find off file prefix: ex {#tracks}_{fileName}

                        // Clear & assign instruments to tracks

                        String fileName = song_Chooser.getSelected();
                        if(fileName == null || fileName.equals("Error")) {
                                System.err.println("******* NO SONG SELECTED *******");
                                return;
                        }

                        int Numberof_Tracks = Integer.parseInt(fileName.split("_")[0]); // Should split off the #_ THEOREITCALLY
                        int motorsPerTrack = 8 / Numberof_Tracks;

                        mOrchestra.clearInstruments();

                        for(int i = 0; i < 8; i++) { 
                                int moduleIndex = i / 2;
                                int track = i / motorsPerTrack; 

                                if(i %  2 == 0) {
                                        TalonFX DriveMotor = drivetrain.getModule(moduleIndex).getDriveMotor();
                                        mOrchestra.addInstrument(DriveMotor,  track);
                                } else {
                                        TalonFX SteerMotor = drivetrain.getModule(moduleIndex).getSteerMotor();
                                        mOrchestra.addInstrument(SteerMotor,  track);
                                }
                        } // ---------------------------------------------

                        mOrchestra.stop();

                        // Play
                        
                        var statusLoad = mOrchestra.loadMusic(fileName);

                        NTHelper.setString("/music/fileName", fileName);

                        if(statusLoad.isOK()) {
                                System.out.println("******* LOADED MUSIC *******");

                                var statusPlay = mOrchestra.play();

                                if(statusPlay.isOK()) {
                                        System.out.println("******* PLAYING MUSIC ******* MUSIC MUSIC MUSIC FCFGAbFGC#C");
                                } else {
                                        System.err.println("******* FAILED TO PLAY MUSIC *******");
                                }
                        } else {
                                System.err.println("******* FAILED TO LOAD MUSIC *******");
                        }

                        // ---------------------------------

                }).ignoringDisable(true);

                Command stopMusic = Commands.runOnce(() -> {
                        mOrchestra.stop();
                }).ignoringDisable(true);

                // Add to dash and Network tables

                SmartDashboard.putData("/music/Play", startMusic);
                SmartDashboard.putData("/music/Stop", stopMusic);

                // NTHelper.setString("/music/File_Name", "Weezer.chrp");
                // NTHelper.setDouble("/music/Numberof_Tracks", 1);
                
                File musicDir = new File(Filesystem.getDeployDirectory(), "music");

                File[] musicFiles = musicDir.listFiles((dir, name) -> name.endsWith(".chrp"));

                if(musicFiles != null) {                       
                        for(File file : musicFiles) {
                                String raw_name = file.getName();
                                song_Chooser.addOption(raw_name, raw_name);
                        }
        
                        song_Chooser.setDefaultOption("Default", "2_The_Duck_Song.chrp");
                } else {
                        song_Chooser.addOption("Error: Failed to find music files", "Error");
                }


                SmartDashboard.putData("/music/Song_Selector", song_Chooser);
        }
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
        
        @Logged
        public final IntakeSubsystem intake = new IntakeSubsystem();

        @Logged
        public final ArmSubsystem arm = new ArmSubsystem();

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(10, 0, 0);
        private Rotation2d lastHeading = new Rotation2d();

        private Supplier<Pose2d> currentPose;

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
        // @Logged
        public final Vision vision = new Vision(currentPose);

        @Logged
        public final HoodSubsystem hood = new HoodSubsystem(this);

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

        @Logged
        private final DashboardTelemetry dashboardlogger = new DashboardTelemetry();
        
        public final RobotHealth robotHealth = new RobotHealth(this);


        @SuppressWarnings("unused")
        private final SubsystemMechanism2d subsystemMechanism2d = new SubsystemMechanism2d(this);

        private final PowerDistribution pdh = new PowerDistribution(30, ModuleType.kRev);

        public RobotContainer() {

                SmartDashboard.putData("subsystems/arm", arm);
                SmartDashboard.putData("subsystems/feederLeft", feederLeft);
                SmartDashboard.putData("subsystems/feederRight", feederRight);
                SmartDashboard.putData("subsystems/intake", intake);
                SmartDashboard.putData("subsystems/shooterLeft", shooterLeft);
                SmartDashboard.putData("subsystems/shooterRight", shooterRight);
                SmartDashboard.putData("subsystems/twindexer", twindexer);
                SmartDashboard.putData("subsystems/hood", hood);
                SmartDashboard.putData("pdh", pdh);

                configureBindings();
                NTHelper.setDouble("/tuning/FeederSpeed", 1);
                NTHelper.setDouble("/tuning/ShooterSpeed", 2800);
                NTHelper.setBoolean("/tuning/snakeMode", false);

        }

        private void configureLeds() {
                kwarqsLed.setDefaultCommand(kwarqsLed.setLeds(() -> {
                        if (!drivetrain.isCameraConnected()) {
                                return "GreenCycle";
                        }
                        if (twindexer.isJammed()) {
                                return "yellow"; // make a yellow cycle
                        }
                        if (intake.isJammed()) {
                                return "orange"; // make an orange cycle
                        }
                        if (drivetrain.isSeeingAprilTag()) {
                                return PoseTransformUtils.isRedAlliance() ? "RedCycle" : "BlueCycle";
                        }
                        if (RobotState.isAutonomous()) {
                                return "YellowAndGreenCycle";
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
                configureShooterTuningControllerBindings();
                configureLeds();

                drivetrain.registerTelemetry(logger::telemeterize);

                RobotModeTriggers.disabled().onTrue(disableEverything());

                // RobotModeTriggers.autonomous().onTrue(hood.hoodDownandReset());
                RobotModeTriggers.teleop().onTrue(hood.hoodDownandReset());
        }

        private void configureDriveControllerBindings() {
                // reset the field-centric heading on left bumper press
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                // Intake commands
                driverController.button(9).whileTrue(intake.outtake()).onFalse(intake.stop());
                driverController.button(10).whileTrue(Commands.parallel(intakeCommands.armDown(),intake.intake()))
                                .onFalse(intake.stop());
                driverController.b().onTrue(arm.armUp());
                driverController.a().onTrue(intakeCommands.armDown());
                // driverController.x().onTrue(hood.set(.1)).onFalse(hood.set(0));
                // driverController.y().onTrue(hood.set(-.1)).onFalse(hood.set(0));

                driverController.rightTrigger(0.25).whileTrue(shooterCommands.prepareToShoot())
                                .onFalse(shooterCommands.stopShooting());

                // Use passing feed command when left trigger is pressed, and shooter feed
                // command otherwise
                Command shooterFeedCommand = shooterCommands.feed();
                Command passingFeedCommand = passingCommands
                                .feedForPassing();
                Command feedCommand = Commands.either(passingFeedCommand, shooterFeedCommand,
                                () -> driverController.leftTrigger().getAsBoolean());

                // Feed when right bumper is pressed
                driverController.rightBumper()
                                .whileTrue(feedCommand)
                                .onFalse(shooterCommands.stopFeeding().andThen(intakeCommands.armDown()));
                // Trench pass when left bumper is pressed
                driverController.leftBumper().whileTrue(passingCommands.trenchPass()).onFalse(intake.stop());
                // Aim to pass on left trigger
                driverController.leftTrigger().whileTrue(passingCommands.aimToPass())
                                .onFalse(shooterCommands.stopShooting());

                driverController.povUp().onTrue(new InstantCommand(() -> shooterCommands.setFixedShootingPose("Auto")));
                driverController.povLeft()
                                .onTrue(new InstantCommand(() -> shooterCommands.setFixedShootingPose("Left Bump")));
                driverController.povRight()
                                .onTrue(new InstantCommand(() -> shooterCommands.setFixedShootingPose("Right Bump")));
                driverController.povDown()
                                .onTrue(new InstantCommand(() -> shooterCommands.setFixedShootingPose("Tower")));

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

                operatorController.leftBumper().whileTrue(twindexer.spindexBack()).onFalse(twindexer.stop());
                operatorController.rightBumper().whileTrue(twindexer.spindex()).onFalse(twindexer.stop());

                operatorController.a().onTrue(hood.hoodDownandReset());

                

                operatorController.button(7).onTrue(new InstantCommand(() -> drivetrain
                                .resetRotation(new Rotation2d(PoseTransformUtils.isRedAlliance() ? 180 : 0))));
                operatorController.button(8).onTrue(new InstantCommand(() -> drivetrain.resetPose(new Pose2d())));

                operatorController.povUp()
                                .onTrue(ShooterCommands.das.increaseVelocityOffset());
                operatorController.povDown()
                                .onTrue(ShooterCommands.das.decreaseVelocityOffset());
        }

        private void configureShooterTuningControllerBindings() {
                shooterTuningController.leftBumper().whileTrue(Commands.waitSeconds(.5).andThen(shooterCommands.feedOnly()))
                                .onFalse(shooterCommands.stopFeeding());

                shooterTuningController.rightBumper()
                                .whileTrue(shooterCommands.rev(() -> NTHelper.getDouble("/tuning/ShooterSpeed", 0)))
                                .onFalse(shooterCommands.stopShooting());
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