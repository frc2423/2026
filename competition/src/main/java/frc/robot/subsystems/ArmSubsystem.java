// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Revolutions;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor = new SparkMax(21, MotorType.kBrushless);

    private final Arm arm;

    private Angle offset = Degrees.of(119 + 5);//Revolutions.of(.735).plus(Degrees.of(30)); // Revolutions.of(Robot.isReal() ? 0.235 + .75 : 0);

    private Angle setpointAngle;
    private double motorPercent = 0;

    public ArmSubsystem() {
        // SparkMaxConfig config = new SparkMaxConfig();
        // armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                // .withEncoderInverted(true)
                // Feedback Constants (PID Constants)
                .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                .withSimClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                // Feedforward Constants
                // Telemetry name and verbosity level
                .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(20))
                .withClosedLoopRampRate(Seconds.of(0.25))
                // Not sure what this should be
                .withGearing(30);

        if (Robot.isReal()) {
            smcConfig.withExternalEncoder(armMotor.getAbsoluteEncoder())
                    .withExternalEncoderInverted(false)
                    .withExternalEncoderGearing(1)
                    .withExternalEncoderZeroOffset(offset)
                    .withUseExternalFeedbackEncoder(true)
                    .withOpenLoopRampRate(Seconds.of(0.25));
        }

        SmartMotorController sparkSmartMotorController = new SparkWrapper(armMotor, DCMotor.getNEO(1), smcConfig);

        ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
                // Soft limit is applied to the SmartMotorControllers PID
                .withSoftLimits(Degrees.of(-10), Degrees.of(100))
                // Hard limit is applied to the simulation.
                .withHardLimit(Degrees.of(-20), Degrees.of(110))
                // Starting position is where your arm starts
                .withStartingPosition(Degrees.of(90))
                // Length and mass of your arm for sim.
                .withLength(Feet.of(1))
                .withMass(Pounds.of(2))
                // Telemetry name and verbosity for the arm.
                .withTelemetry("Arm", TelemetryVerbosity.HIGH);

        arm = new Arm(armCfg);
    }

    ArmFeedforward feedforward = new ArmFeedforward(0, .05, .45);

    public Command armUp() {
        return setAngle(Degrees.of(80)).withName("armUp");
    }

    public Command setAngle(Angle angle) {
        return runOnce(() -> {
            setpointAngle = angle;
        }).withName("setArmAngle");
    }

    public Command wiggleArm(Angle angle1, Angle angle2, Time time) {
        return Commands.repeatingSequence(
                setAngle(angle1),
                Commands.waitSeconds(time.in(Seconds)),
                setAngle(angle2),
                Commands.waitSeconds(time.in(Seconds))).withName("wiggleArm");
    }

    public Command set(double dutycycle) {
        return runOnce(() -> {
            motorPercent = dutycycle;
            setpointAngle = null;
        }).withName("setArmSpeed");
    }

    @Override
    public void periodic() {
        arm.updateTelemetry();

        double motorSpeed = 0;

        if (setpointAngle != null) {
            Angle armAngle = arm.getAngle();
            if (armAngle.gt(Degrees.of(300))) {
                armAngle = armAngle.minus(Degrees.of(360));
            }
            double armAngleRadians = armAngle.in(Radians);
            double setAngleRadians = setpointAngle.in(Radians);
            motorSpeed = feedforward.calculate(armAngleRadians, setAngleRadians - armAngleRadians);
        } else {
            motorSpeed = motorPercent;
        }
        if (motorSpeed < -.4) {
            motorSpeed = -.4;
        }
        armMotor.set(motorSpeed);
    }

    public SparkMax getSparkMax(){
        return this.armMotor;
    }

    @Override
    public void simulationPeriodic() {
       arm.simIterate();
    }

    @Logged
    public double getAngle() {
        return arm.getAngle().in(Degrees);
        // return 0;
    }

    @Logged
    public double getVoltage() {
       return arm.getMotor().getVoltage().in(Volts);
    //    return 0;
    }

    @Logged
    public double getSetpoint() {
        if (setpointAngle != null) {
            return setpointAngle.in(Degrees);
        }
        return 0;
    }

    @Logged
    public double getMotorPercent() {
       return armMotor.get();
    //    return 0;
    }

    public boolean isNear(Angle angle, Angle tolerance) {
        return arm.getAngle().isNear(angle, tolerance);
        // return true;
    }

    public boolean isDown() {
        return isNear(Degrees.of(15), Degrees.of(20));
        // return true;
    }
}
