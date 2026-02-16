package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class FeederSubsystem extends SubsystemBase {

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0018);
    private double shootSpeed = 1;
    private boolean isInverted;
    
    public SparkFlex motor;

    public FeederSubsystem (int motorId, boolean isInverted) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);
        this.isInverted = isInverted;

        SparkFlexConfig config = new SparkFlexConfig();
        // config.closedLoop.p(.002).i(0).d(.04).outputRange(-1,1 );
        config.inverted(isInverted);
        config.closedLoop.p(0.001).i(0).d(0).outputRange(-1,1 );
        config.smartCurrentLimit(80,80);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        setDefaultCommand(stop());

        NTHelper.setDouble("/shooter/speed", 1);

        
    }

    


    public Command spin(){
        return run(()->{
            motor.set(1);
        });
    }

    public Command spin(Supplier<Double> value) {
        return run(() -> {

            motor.set(value.get());

        });
    }

    public boolean isRevved() {
        return getVelocity() > 2000;
    }

    public Command stop(){
        return run (()->{
            motor.stopMotor();
        });
    } 

    public Command spinWithSetpoint(Supplier<Double> setpoint){

        double setpointYAY = setpoint.get();

         return run(()->{
            double voltage = feedforward.calculate(setpointYAY);
            motor.getClosedLoopController().setReference(setpointYAY, ControlType.kVelocity, ClosedLoopSlot.kSlot0, voltage);
        });

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
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("encoderspeed", () -> motor.getEncoder().getVelocity(), null);
        builder.addDoubleProperty("motorSpeed", () -> motor.get(), null);

    }
   

}