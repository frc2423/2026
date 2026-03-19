package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;

public class FeederSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private double motorVoltage = 0;

    public FeederSubsystem(int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(isInverted);
        config.openLoopRampRate(.5);
        config.smartCurrentLimit(80, 80);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        NTHelper.setDouble("/shooter/speed", 1);
    }

    public Command spin() {
        return runOnce(() -> {
            motorVoltage = 8;
        }).withName("spinFeeder");
    }

    // public Command spin(Supplier<Double> value) {
    //     return run(() -> {
    //         percentSpeed = .7;
    //     }).withName("spinFeeder");
    // }

    public Command stop() {
        return runOnce(() -> {
            motorVoltage = 0;
        }).withName("stopFeeder");
    }

    @Override
    public void periodic() {
        motor.setVoltage(motorVoltage);
    }

    @Logged
    public double getMotorSpeed() {
        return motor.get();
    }

    @Logged
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("encoderspeed", () -> motor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("motorSpeed", () -> motor.get(), null);

    }
}