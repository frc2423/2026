package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.subsystems.TwindexerIOInputsAutoLogged;

public class Twindexer {
    private SparkFlex motor = new SparkFlex(23, MotorType.kBrushless);
    private final SparkFlexConfig motorConfig = new SparkFlexConfig();
    private final TwindexerIO io;
    private final TwindexerIOInputsAutoLogged inputs;
    private final int index;

    public Twindexer(TwindexerIO io, int index) {
        this.io = io;
        this.index = index;

        motorConfig.openLoopRampRate(.1);
        motorConfig.idleMode(IdleMode.kCoast);
        motorConfig.inverted(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Twindexer/" + Integer.toString(index), inputs);
    }
    
    public SparkFlex getSparkFlex() {
        return this.motor;
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getSpeed() {
        return motor.get();
    }

    public double getOutputCurrent() {
        return motor.getOutputCurrent();
    }
    
    public boolean isJammed() {
        return motor.getOutputCurrent() > 90 && motor.getEncoder().getVelocity() < 100;
    }

}
