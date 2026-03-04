// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {

    private final SparkMax hoodMotorBase = new SparkMax(32, MotorType.kBrushless);
    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // .withFeedforward(new SimpleMotorFeedforward(1, 1))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(1, 0, 0)
            .withSimClosedLoopController(0, 0, 0)
            // Feedforward Constants
            // Telemetry name and verbosity level
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withGearing(20 * 85 / 10.0)
            .withOpenLoopRampRate(Seconds.of(0.25));

    private final SmartMotorController hoodMotor = new SparkWrapper(hoodMotorBase, DCMotor.getNeo550(1), smcConfig);

    private double p = 0.03;

    private final PIDController hoodPidController = new PIDController(p, 0, 0);

    private Angle setpointAngle;
    private double motorPercent = 0;

    public HoodSubsystem() {
        // hoodMotor.close
        SmartDashboard.putData("/dashboardCommands/bumpUp5Degrees", bumpUp5Degrees());
        SmartDashboard.putData("/dashboardCommands/bumpDown5Degrees", bumpDown5Degrees());
        SmartDashboard.putData("/dashboardCommands/setZero", setEncoderPosition(Degrees.of(0)));
        SmartDashboard.putData("/dashboardCommands/hoodPid", hoodPidController);
        // SmartDashboard.getNumber("/dashboardCommands/hoodPID", p);


    }

    public Command hoodUp() {
        return setAngle(Degrees.of(45)).withName("hoodUp");
    }

    public Command setAngle(Angle angle) {
        return runOnce(() -> {
            setpointAngle = angle;
        }).withName("setHoodAngle");
    }

    public Command setAngle(Supplier<Angle> setpoint) {
        return run(() -> {
            setpointAngle = setpoint.get();
        }).withName("setHoodAngle");
    }

    public Command set(double dutycycle) {
        return runOnce(() -> {
            motorPercent = dutycycle;
            setpointAngle = null;
        }).withName("setHoodSpeed");
    }

    public Command bumpUp5Degrees() {
        return runOnce(() -> {
            setpointAngle = Degrees.of(getAngle() + 5);
        }).withName("bumpUp5Degrees");

    }

    public Command bumpDown5Degrees() {
        return runOnce(() -> {
            setpointAngle = Degrees.of(getAngle() - 5);
        }).withName("bumpUp5Degrees");
    }

    @Override
    public void periodic() {
        hoodMotor.updateTelemetry();

        if (setpointAngle != null) {
            double calcedMotorPercent = hoodPidController.calculate(getAngle(), setpointAngle.in(Degrees));
            hoodMotor.setDutyCycle(calcedMotorPercent);

        } else {
            hoodMotor.setDutyCycle(motorPercent);
        }

    }

    @Override
    public void simulationPeriodic() {
        hoodMotor.simIterate();
    }

    public Command setEncoderPosition(Angle angler) {
        return runOnce(() -> {
            hoodMotor.setEncoderPosition(angler);
            setpointAngle = angler;
        });
    }

    @Logged
    public double getAngle() {
        return hoodMotor.getMechanismPosition().in(Degrees);
    }

    @Logged
    public double getVoltage() {
        return hoodMotor.getVoltage().in(Volts);
    }

    @Logged
    public double getSetpoint() {
        // Angle angle = hoodMotor.getMechanismPositionSetpoint().orElse(Degrees.of(0));
        if (setpointAngle != null) {
            return setpointAngle.in(Degrees);
        }
        return 0;
    }

    @Logged
    public double getMotorPercent() {
        return hoodMotorBase.get();
    }

    public boolean isNear(Angle angle, Angle tolerance) {
        return hoodMotor.getMechanismPosition().isNear(angle, tolerance);
    }

    public boolean isDown() {
        return isNear(Degrees.of(0), Degrees.of(20));
    }
}
