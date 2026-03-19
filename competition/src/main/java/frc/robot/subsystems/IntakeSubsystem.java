package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex motor = new SparkFlex(22, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private double percentSpeed = 0;

    public IntakeSubsystem() {
        motorConfig.inverted(true);
        setCurrentLimit(100, 100);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command intake() {
        return runOnce(() -> {
            percentSpeed = 1;
        }).withName("intake");
    }
    
    public Command intakeSlow() {
        return runOnce(() -> {
            percentSpeed = .5;
        }).withName("intakeSlow");
    }

    public Command outtake() {
        return runOnce(() -> {
            percentSpeed = -1;
        }).withName("outtake");
    }

    public Command outtakeDown() {
        return runOnce(() -> {
            percentSpeed = -.2;
        }).withName("outtakeDown");
    }

    public Command stop() {
        return runOnce(() -> {
            percentSpeed = 0;
        }).withName("intakeStop");
    }

    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {
        motor.set(percentSpeed);
    }

    public boolean isJammed() {
        return motor.getOutputCurrent() > 90 && motor.getEncoder().getVelocity() < 100;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    }

}