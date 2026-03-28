package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
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

    // private final SimpleMotorFeedforward feedforward = new
    // SimpleMotorFeedforward(0, 0.0018);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0018);
    private final SparkFlex motor;
    private double shooterSetpoint = 0;

    public ShooterSubsystem(int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();

        config.encoder.velocityConversionFactor(1)
                .positionConversionFactor(1)
                .uvwMeasurementPeriod(8)
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(8);

        config.closedLoopRampRate(.1)
                .openLoopRampRate(.1)
                .inverted(isInverted)
                .idleMode(IdleMode.kCoast);

        config.idleMode(IdleMode.kCoast);
        config.closedLoop
                .p(1000.1)
                // .p(0.01)
                // .p(0.0)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                .allowedClosedLoopError(75, ClosedLoopSlot.kSlot0);

        REVLibError error = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        System.out.println("Shooter " + motorId + " error:" + error.name());

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

    boolean isOn = false;

    @Override
    public void periodic() {

        if (shooterSetpoint == 0) {
            motor.stopMotor();
        } else {
            // double velocity = getVelocity();
            // boolean isAboveSetpoint = velocity > (shooterSetpoint + 50);
            // boolean isBelowSetpoint = velocity < (shooterSetpoint - 50);

            // if (isAboveSetpoint) {
            // isOn = false;
            // } else if (isBelowSetpoint) {
            // isOn = true;
            // }
            // double voltage = feedforward.calculate(shooterSetpoint);

            // motor.set(isOn ? 1 : voltage);

            // if (getVelocity() > )
            // double signedSetpoint = isInverted ? -shooterSetpoint : shooterSetpoint;
            double voltage = feedforward.calculate(shooterSetpoint);
            motor.getClosedLoopController().setReference(shooterSetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0, voltage);
        }
    }

    @Logged
    public double getFeedforward() {
        return feedforward.calculate(shooterSetpoint);
    }

    @Logged
    public double getVelocity() {
        double velocity = motor.getEncoder().getVelocity();
        // return isInverted ? -velocity : velocity;
        return velocity;
    }

    @Logged
    public double getMotorSpeed() {
        double speed = motor.get();
        // return isInverted ? -speed : speed;
        return speed;
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