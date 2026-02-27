package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwindexerSubsystem extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(2);
    private double percentSpeed = 0;

    public TwindexerSubsystem() {
        setCurrentLimit(100, 80);
        motorConfig.idleMode(IdleMode.kCoast);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command spindex() {
        return runOnce(() -> {
            percentSpeed = 1;
        }).withName("spindex");
    }

    public Command spindexBack() {
        return runOnce(() -> {
            percentSpeed = -0.5;
        }).withName("spindexBack");
    }

    public Command stop() {
        return runOnce(() -> {
            percentSpeed = 0;
        }).withName("stopSpindexer");
    }

    @Override
    public void periodic() {
        motor.set(speedLimiter.calculate(percentSpeed));

    }

    @Logged
    public boolean isJammed() {
        return motor.getOutputCurrent() > 90 && motor.getEncoder().getVelocity() < 100;
    }

    @Logged
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Logged
    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }

    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    }

}