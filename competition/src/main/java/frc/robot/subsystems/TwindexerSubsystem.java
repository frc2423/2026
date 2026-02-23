package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TwindexerSubsystem extends SubsystemBase {

    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(2);

    private final Mechanism2d mechanism2d = new Mechanism2d(2, 1);

    private final MechanismLigament2d[] mechanismLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };
    private final MechanismLigament2d[] mechanismLigaments2 = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };
    private final MechanismLigament2d[][] allLigaments = { mechanismLigaments, mechanismLigaments2 };

    public TwindexerSubsystem() {
        setCurrentLimit(100, 80);
        motorConfig.idleMode(IdleMode.kCoast);

        setDefaultCommand(stop());

        addMechanism2dRoot(0, .55);
        addMechanism2dRoot(1, .7);

        SmartDashboard.putData("Mechanism2ds/Twindexer", mechanism2d);
    }

    private void addMechanism2dRoot(int index, double x) {
        var root = mechanism2d.getRoot("twindexer" + index, x, .3);
        for (int i = 0; i < allLigaments[index].length; i++) {
            var ligament = allLigaments[index][i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(0, 255, 0) : new Color8Bit(0, 0, 255);
            ligament.setColor(color);
            root.append(ligament);
        }
    }

    private void setCurrentLimit(int stallLimit, int freeLimit) {
        motorConfig.smartCurrentLimit(stallLimit, freeLimit);
        motor.configureAsync(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command spindex() {
        return run(() -> {
            motor.set(speedLimiter.calculate(0.5));
        });
    }

    public Command spindexBack() {
        return run(() -> {
            motor.set(speedLimiter.calculate(-0.5));
        });
    }

    public Command stop() {
        return run(() -> {
            motor.set(speedLimiter.calculate(0));
        });
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

    @Override
    public void periodic() {
        double percentSpeed = motor.get();
        for (int i = 0; i < allLigaments.length; i++) {
            for (var ligament : allLigaments[i]) {
                double angleChange = i == 0 ? -percentSpeed * 10 : percentSpeed * 10;
                ligament.setAngle(ligament.getAngle() + angleChange);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("current", () -> motor.getOutputCurrent(), null);
        builder.addDoubleProperty("motor_speed", () -> motor.get(), null);

    } // -147.353760 start /

}