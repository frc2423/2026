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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;


public class ShooterSubsystem extends SubsystemBase {

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.0018);
    private double shootSpeed = 8;
    private CommandSwerveDrivetrain drivebase;
    
    public SparkFlex motor;

    public ShooterSubsystem (int motorId) {
        motor = new SparkFlex(motorId, MotorType.kBrushless);
        SparkFlexConfig motorConfig = new SparkFlexConfig();

        SparkFlexConfig config = new SparkFlexConfig();
        // config.closedLoop.p(.002).i(0).d(.04).outputRange(-1,1 );
        config.closedLoop.p(0.001).i(0).d(0).outputRange(-1,1 );

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        NTHelper.setDouble("/shooter/speed", 0);
    }


    public Command spin(){
        return run(()->{
            double speed = NTHelper.getDouble("/shooter/speed", 0);
            double voltage = feedforward.calculate(speed);
            //motor.set(-.25);
            motor.getClosedLoopController().setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, voltage);
        });
        // motor.
        // motorConfig.
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

    

    public Command rev() {
        return run (()-> {
            motor.set(shootSpeed);

        });
    }
 
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("encoderspeed", () -> motor.getEncoder().getVelocity(), null);

    }
   

}