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
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.thethriftybot.devices.ThriftyNova;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;

public class ArmSubsystem extends SubsystemBase {

    private final SparkMax armMotor = Robot.isReal() ? new SparkMax(21, MotorType.kBrushless) : null;
    private final ThriftyNova armMotorSim = Robot.isSimulation() ? new ThriftyNova(21) : null;

    private final Arm arm;

    public ArmSubsystem() {
        SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                // Feedback Constants (PID Constants)
                .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
                // Feedforward Constants
                // .withFeedforward(new ArmFeedforward(0, 0, 0))
                // .withSimFeedforward(new ArmFeedforward(0, 0, 0))
                // Telemetry name and verbosity level
                .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(40))
                .withClosedLoopRampRate(Seconds.of(0.25))
                // Not sure what this should be
                .withGearing(30);

        if (Robot.isReal()) {
            smcConfig.withExternalEncoder(armMotor.getAbsoluteEncoder())
                    .withExternalEncoderInverted(true)
                    .withExternalEncoderGearing(1)
                    .withExternalEncoderZeroOffset(Revolutions.of(0.235))
                    .withUseExternalFeedbackEncoder(true)
                    .withOpenLoopRampRate(Seconds.of(0.25));
            // Motor properties to prevent over currenting.
            // .withMotorInverted(true)
        }

        SmartMotorController sparkSmartMotorController = Robot.isReal()
                ? new SparkWrapper(armMotor, DCMotor.getNEO(1), smcConfig)
                : new NovaWrapper(armMotorSim, DCMotor.getNEO(1), smcConfig);

        ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
                // Soft limit is applied to the SmartMotorControllers PID
                .withSoftLimits(Degrees.of(80), Degrees.of(190))
                // Hard limit is applied to the simulation.
                .withHardLimit(Degrees.of(70), Degrees.of(200))
                // Starting position is where your arm starts
                .withStartingPosition(Degrees.of(180))
                // Length and mass of your arm for sim.
                .withLength(Feet.of(1))
                .withMass(Pounds.of(2))
                // Telemetry name and verbosity for the arm.
                .withTelemetry("Arm", TelemetryVerbosity.HIGH);

        arm = new Arm(armCfg);
    }

    public Command setAngle(Angle angle) {
        return arm.set(() -> {
            double armAngle = arm.getAngle().in(Rotations);
            double setAngle = angle.in(Rotations);
            System.out.println("angle error: " + setAngle + ", " + armAngle + ", " + (setAngle - armAngle));
            return (setAngle - armAngle);
        });
        // return arm.run(angle);
        // return arm.setAngle(angle);
    }

    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    @Override

    public void periodic() {
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();
    }

    @Logged
    public double getAngle() {
        return arm.getAngle().in(Degrees);
    }

    @Logged
    public double getVoltage() {
        return arm.getMotor().getVoltage().in(Volts);
    }

    @Logged
    public double getSetpoint() {
        Angle angle = arm.getMechanismSetpoint().orElse(Degrees.of(0));
        return angle.in(Degrees);
    }
}
