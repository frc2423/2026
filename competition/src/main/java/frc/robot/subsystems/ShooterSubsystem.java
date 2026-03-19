package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
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

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0018);
    private final SparkFlex motor;
    private double shooterSetpoint = 0;

    public ShooterSubsystem(int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.encoder.velocityConversionFactor(1);
        config.encoder.positionConversionFactor(1);
        config.encoder.uvwMeasurementPeriod(8)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(8);
        config.closedLoopRampRate(.2);
        config.openLoopRampRate(.2);
        config.inverted(isInverted);
        // config.encoder.countsPerRevolution(1);
        config.idleMode(IdleMode.kCoast);
        // config.closedLoop.p(.002).i(0).d(.04).outputRange(-1,1 );
        if (!isInverted) {
            config.closedLoop.p(100000.5).i(0).d(0).outputRange(0, 1);
        } else {
            config.closedLoop.p(100000.5).i(0).d(0).outputRange(-1, 0);

        }
        config.closedLoop.allowedClosedLoopError(50, ClosedLoopSlot.kSlot0);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        NTHelper.setDouble("/shooter/speed", 2800);
    }

    public Command stop() {
        return runOnce(() -> {
            shooterSetpoint = 0;
        }).withName("stopShooter");
    }

    public Command spinWithSetpoint(Supplier<Double> setpoint) {
        return run(() -> {
            shooterSetpoint = setpoint.get();
        }).withName("spinWithSetpoint");
    }

    public Command spinWithSetpoint(double setpoint) {
        return runOnce(() -> {
            shooterSetpoint = setpoint;
        }).withName("spinWithSetpoint");
    }

    @Override
    public void periodic() {

        if (shooterSetpoint == 0) {
            motor.stopMotor();
        } else {
            double voltage = feedforward.calculate(shooterSetpoint);
            motor.getClosedLoopController().setReference(shooterSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                    voltage);
        }
    }

    @Logged
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Logged
    public double getMotorSpeed() {
        return motor.get();
    }

    @Logged
    public double getSetpoint() {
        return shooterSetpoint;
        // return motor.getClosedLoopController().getSetpoint();
    }

    @Logged
    public boolean isAtSetpoint() {
        return Math.abs(getVelocity() - getSetpoint()) <= 50;
        // return motor.getClosedLoopController().isAtSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("encoderspeed", () -> motor.getEncoder().getVelocity(), null);

    }

}