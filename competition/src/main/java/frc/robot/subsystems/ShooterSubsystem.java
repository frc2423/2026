package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;

public class ShooterSubsystem extends SubsystemBase {

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0018);
    private double shootSpeed = 8;

    public SparkFlex motor;
    private boolean isInverted;

    public ShooterSubsystem(int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);
        this.isInverted = isInverted;

        SparkFlexConfig config = new SparkFlexConfig();
        config.encoder.velocityConversionFactor(1);
        config.encoder.positionConversionFactor(1);
        config.inverted(isInverted);
        // config.encoder.countsPerRevolution(1);
        config.idleMode(IdleMode.kCoast);
        // config.closedLoop.p(.002).i(0).d(.04).outputRange(-1,1 );
        config.closedLoop.p(0.003).i(0).d(0).outputRange(-1, 0);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        setDefaultCommand(stop());

        NTHelper.setDouble("/shooter/speed", 2800);
    }

    public Command spin() {
        return run(() -> {
            double speed = NTHelper.getDouble("/shooter/speed", 0);
            double voltage = feedforward.calculate(speed);
            // motor.set(-.25);
            motor.getClosedLoopController().setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, voltage);
        });
        // motor.
        // motorConfig.
    }

    public Command stop() {
        return run(() -> {
            motor.stopMotor();
        });
    }

    public Command spinWithSetpoint(Supplier<Double> setpoint) {

        return run(() -> {
            double setpointYAY = setpoint.get();
            double voltage = feedforward.calculate(setpointYAY);
            motor.getClosedLoopController().setReference(setpointYAY, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                    voltage);
        });

    }

    public Command rev() {
        return run(() -> {
            motor.set(shootSpeed);

        });
    }

    public Command rev(Supplier<Double> speed) {
        return run(() -> {
            motor.set(speed.get());

        });
    }

    @Logged
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }
  
    @Logged
    public double getMotorSpeed() {
        return motor.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("encoderspeed", () -> motor.getEncoder().getVelocity(), null);

    }

}