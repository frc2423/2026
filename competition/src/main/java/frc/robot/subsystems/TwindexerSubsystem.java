package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwindexerSubsystem extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private double motorPercent = 0;

    public TwindexerSubsystem() {
        motorConfig.openLoopRampRate(.1);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.inverted(true);
        setCurrentLimit(100, 80);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command spindex() {
        return runOnce(() -> {
            motorPercent = 1;
        }).withName("spindex");
    }

    public SparkFlex getSparkFlex(){

        return this.motor;

    }

    public Command spindexBack() {
        return runOnce(() -> {
            motorPercent = -1;
        }).withName("spindexBack");
    }

    public Command stop() {
        return runOnce(() -> {
            motorPercent = 0;
        }).withName("stopSpindexer");
    }

    @Override
    public void periodic() {
        motor.set(motorPercent);

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

    @Logged
    public double getInputCurrent() {
        return Math.abs(motor.getOutputCurrent() * motor.getAppliedOutput());
    }

    @Logged
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