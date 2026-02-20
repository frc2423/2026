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

    private SparkFlex motor = new SparkFlex(22, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    private double percentSpeed = 0;

    public IntakeSubsystem() {
        setCurrentLimit(100, 100);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command intake() {
        return runOnce(() -> {
            percentSpeed = 1;
        });
    }

    public Command outtake() {
        return runOnce(() -> {
            percentSpeed = -1;
        });
    }

    public Command stop() {
        return runOnce(() -> {
            percentSpeed = 0;
        });
    }

    public Command setMotorSpeed(double speed) {
        return runOnce(()-> {
            percentSpeed = speed;
        });
    }

    @Override
    public void periodic() {
        motor.set(percentSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    } // -147.353760 start /

}