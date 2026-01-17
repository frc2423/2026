package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);

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
        return run(() -> {
            motor.stopMotor();
        });
    }

}
