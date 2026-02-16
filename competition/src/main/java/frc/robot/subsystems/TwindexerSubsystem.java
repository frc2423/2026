package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwindexerSubsystem extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(2);

    public TwindexerSubsystem() {
        setCurrentLimit(80, 80);
        motorConfig.idleMode(IdleMode.kCoast);

        setDefaultCommand(stop());
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command spindex() {
        return run(() -> {
            motor.set(speedLimiter.calculate(0.5));
        });
    }

    public Command spindexBack() {
        return run(() -> {
            motor.set(speedLimiter.calculate(-0.5));
        });
    }

    public Command stop() {
        return run(() -> {
            motor.set(speedLimiter.calculate(0));
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    } // -147.353760 start /

}