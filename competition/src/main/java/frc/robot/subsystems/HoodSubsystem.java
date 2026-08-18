// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
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
            .withStatorCurrentLimit(Amps.of(30))
            .withMotorInverted(true)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withGearing(20 * 85 / 10.0)
            .withOpenLoopRampRate(Seconds.of(0.25));

    private final SmartMotorController hoodMotor = new SparkWrapper(hoodMotorBase, DCMotor.getNeo550(1), smcConfig);

    private final PIDController hoodPidController = new PIDController(0.02, 0, 0);
    private final RobotContainer robot;

    private Angle setpointAngle;
    private double motorPercent = 0;

    private static final int CURRENT_FILTER_SIZE = 30;
    private final MedianFilter currentFilter = new MedianFilter(CURRENT_FILTER_SIZE);

    public HoodSubsystem(RobotContainer robot) {
        this.robot = robot;
        SmartDashboard.putData("subsystems/hood/setZero", setEncoderPosition(Degrees.of(0)));
        SmartDashboard.putData("subsystems/hood/hoodDownandReset", hoodDownandReset());
        NTHelper.setDouble("/SmartDashboard/subsystems/hood/desiredAngle", 0);
        SmartDashboard.putData("subsystems/hood/updateSetpoint", runOnce(() -> {
            double desiredAngle = NTHelper.getDouble("/SmartDashboard/subsystems/hood/desiredAngle", 0);
            setpointAngle = Degrees.of(desiredAngle);
        }).ignoringDisable(true));
    }

    public Command hoodDown() {
        return setAngle(Degrees.of(0)).withName("hoodDown");
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

    public Current getCurrent() {
        Current current = hoodMotor.getStatorCurrent();
        if (current == null) {
            return Amps.of(0);
        }
        return current;
    }

    public SparkMax getSparkMax(){
        return this.hoodMotorBase;
    }

    @Logged
    public double getCurrentInAmps() {
        return getCurrent().in(Amps);
    }

    @Logged
    public double getSampledCurrentInAmps() {
        return currentFilter.calculate(getCurrentInAmps());
    }

    @Logged
    public boolean isStalled() {
        return getSampledCurrentInAmps() > 25;
    }

    public Command hoodDownandReset() {
        return Commands.sequence(
                set(-.1),
                Commands.waitUntil(() -> isStalled()).withTimeout(1.5),
                set(0),
                Commands.waitSeconds(.5),
                setEncoderPosition(Degrees.of(0)));
    }

    public boolean isHoodSafeToDeploy() {
        var robotSpeeds = robot.drivetrain.getState().Speeds;
        double robotVelocity = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        return robot.feederLeft.getMotorSpeed() > 0 || robotVelocity < .1;
    }

    @Override
    public void periodic() {
        hoodMotor.updateTelemetry();

        if (setpointAngle == null) {
            hoodMotor.setDutyCycle(motorPercent);

            if (motorPercent == 0 && !isHoodSafeToDeploy()) {
                setpointAngle = Degrees.of(0);
            }
            return;
        }
        

        if (!isHoodSafeToDeploy()) {
            setpointAngle = Degrees.of(0);
        }

        double calcedMotorPercent = hoodPidController.calculate(getAngle(), setpointAngle.in(Degrees));
        hoodMotor.setDutyCycle(calcedMotorPercent);

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
