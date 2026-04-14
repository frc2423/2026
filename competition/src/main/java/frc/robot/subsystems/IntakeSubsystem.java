package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

   private final SparkFlex motor = new SparkFlex(24, MotorType.kBrushless);
    private final SparkFlex motor2 = new SparkFlex(38, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final SparkFlexConfig motorConfig2 = new SparkFlexConfig();
    private double percentSpeed = 0;

    private static final int CURRENT_FILTER_SIZE = 30;
    private final MedianFilter currentFilter = new MedianFilter(CURRENT_FILTER_SIZE);

    public IntakeSubsystem() {
        motorConfig.inverted(true);
        motorConfig.smartCurrentLimit(60, 60);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.openLoopRampRate(.1);
       
        motorConfig2.inverted(false);
        motorConfig2.smartCurrentLimit(60, 60);
        motorConfig2.idleMode(IdleMode.kCoast);
        motorConfig2.openLoopRampRate(.1);
        
       motor.configureAsync(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configureAsync(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intake() {
        return runOnce(() -> {
            percentSpeed = 1;
        }).withName("intake");
    }
    
    public Command intakeSlow() {
        return runOnce(() -> {
            percentSpeed = .5;
        }).withName("intakeSlow");
    }

    public Command outtake() {
        return runOnce(() -> {
            percentSpeed = -1;
        }).withName("outtake");
    }

    public Command outtakeDown() {
        return runOnce(() -> {
            percentSpeed = -.2;
        }).withName("outtakeDown");
    }

    public Command stop() {
        return runOnce(() -> {
            percentSpeed = 0;
        }).withName("intakeStop");
    }

    public double getSpeed() {
        return motor.get();
        // return 0;
    }

    @Override
    public void periodic() {
       motor.set(percentSpeed * .75);
       motor2.disable();
        // motor2.set(percentSpeed * .75);
    }

    public boolean isJammed() {
        return motor.getOutputCurrent() > 90 && motor.getEncoder().getVelocity() < 100;
        // return false;
    }

    public SparkFlex getSparkFlex1(){
        return this.motor;
    }
    public SparkFlex getSparkFlex2(){
        return this.motor2;
    }

    public double getCurrentInAmps() {
        return motor.getOutputCurrent();
    }

    public double getSampledCurrentInAmps() {
        return currentFilter.calculate(getCurrentInAmps());
    }

    public boolean isStalled() {
        return getSampledCurrentInAmps() > 10;
    }

    @Logged
    public double getPercentSpeed() {
        return percentSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    }

}