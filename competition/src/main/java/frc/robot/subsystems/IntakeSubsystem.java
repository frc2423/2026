package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex motor = new SparkFlex(22, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();

    private final Mechanism2d mechanism2d = new Mechanism2d(1, 1);

    private final MechanismLigament2d[] mechanismLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };

    public IntakeSubsystem() {
        setCurrentLimit(100, 100);

        var root = mechanism2d.getRoot("intake", .8, .3);
        for (int i = 0; i < mechanismLigaments.length; i++) {
            var ligament = mechanismLigaments[i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(255, 0, 0) : new Color8Bit(0, 255, 0);
            ligament.setColor(color);
            root.append(ligament);
        }

        SmartDashboard.putData("Mechanism2ds/Intake", mechanism2d);
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command intake() {
        return run(() -> {
            motor.set(1);
        });
    }

    public Command outtake() {
        return run(() -> {
            motor.set(-1);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            motor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        double percentSpeed = motor.get();
        for (var ligament : mechanismLigaments) {
            ligament.setAngle(ligament.getAngle() - percentSpeed * 10);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    } // -147.353760 start /

}