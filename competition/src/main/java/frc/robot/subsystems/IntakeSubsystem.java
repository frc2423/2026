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

    private SparkFlex motor = new SparkFlex(2, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();

    public IntakeSubsystem() {
        setCurrentLimit(80, 80);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command spin() {
        return run(() -> {
            motor.set(-.5);
        });
    }

    public Command outtake() {
        return run(() -> {
            motor.set(.5);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.stopMotor();
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    } // -147.353760 start /

}