package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;

public class FeederSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private double motorPercent = 0;

    public FeederSubsystem(int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(isInverted);
        config.openLoopRampRate(.1);
        config.smartCurrentLimit(80, 80);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        NTHelper.setDouble("/shooter/speed", 1);
    }

    public Command spin() {
        return runOnce(() -> {
            motorPercent = 1;
        }).withName("spinFeeder");
    }

    // public Command spin(Supplier<Double> value) {
    //     return run(() -> {
    //         percentSpeed = .7;
    //     }).withName("spinFeeder");
    // }

    public Command stop() {
        return runOnce(() -> {
            motorPercent = 0;
        }).withName("stopFeeder");
    }

    public SparkFlex getSparkFlex(){
        return this.motor;
    }

    @Override
    public void periodic() {
        motor.set(motorPercent);
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