package frc.robot.telemetry;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;

public class SubsystemMechanism2d {

    private final RobotContainer robot;
    private Notifier notifier = new Notifier(this::update);

    private final Mechanism2d mechanism2d = new Mechanism2d(2, 1);

    private final MechanismLigament2d[] intakeLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };

    private final MechanismLigament2d[] leftShooterLigaments = {
            new MechanismLigament2d("part1", 0.05, 45),
            new MechanismLigament2d("part2", 0.05, 135),
            new MechanismLigament2d("part3", 0.05, 225),
            new MechanismLigament2d("part4", 0.05, 315),
    };

    private final MechanismLigament2d[] hoodLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
    };

    private final MechanismLigament2d[] leftSpindexerLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };
    private final MechanismLigament2d[] rightSpindexerLigaments = {
            new MechanismLigament2d("part1", 0.05, 0),
            new MechanismLigament2d("part2", 0.05, 90),
            new MechanismLigament2d("part3", 0.05, 180),
            new MechanismLigament2d("part4", 0.05, 270),
    };
    private final MechanismLigament2d[][] twindexerLigaments = { leftSpindexerLigaments, rightSpindexerLigaments };

    public SubsystemMechanism2d(RobotContainer robot) {
        this.robot = robot;

        SmartDashboard.putData("mechanism2ds", mechanism2d);

        addIntake();
        addHood();
        addShooter("leftShooter", leftShooterLigaments);
        addSpindexer(0, .55);
        addSpindexer(1, .7);

        notifier.startPeriodic(.02);

    }

    private void addIntake() {
        var root = mechanism2d.getRoot("intake", 1.4, .3);
        for (int i = 0; i < intakeLigaments.length; i++) {
            var ligament = intakeLigaments[i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(255, 0, 0) : new Color8Bit(0, 255, 0);
            ligament.setColor(color);
            root.append(ligament);
        }
    }

    private void addHood() {
        var root = mechanism2d.getRoot("hood", 0.4, 0.8);
        for (int i = 0; i < hoodLigaments.length; i++) {
            var ligament = hoodLigaments[i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(255, 0, 0) : new Color8Bit(0, 255, 0);
            ligament.setColor(color);
            root.append(ligament);
        }
    }

    private void addShooter(String name, MechanismLigament2d[] ligaments) {
        var root = mechanism2d.getRoot(name, .4, 0.7);
        for (int i = 0; i < ligaments.length; i++) {
            var ligament = ligaments[i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(0, 200, 100) : new Color8Bit(255, 100, 0);
            ligament.setColor(color);
            root.append(ligament);
        }
    }

    private void addSpindexer(int index, double x) {
        var root = mechanism2d.getRoot("twindexer" + index, x, .3);
        for (int i = 0; i < twindexerLigaments[index].length; i++) {
            var ligament = twindexerLigaments[index][i];
            ligament.setLineWeight(1);
            var color = i % 2 == 0 ? new Color8Bit(0, 255, 0) : new Color8Bit(0, 0, 255);
            ligament.setColor(color);
            root.append(ligament);
        }
    }

    private void update() {
        for (var ligament : intakeLigaments) {
            ligament.setAngle(ligament.getAngle() - robot.intake.getSpeed() * 10);
        }

        double twindexerSpeed = robot.twindexer.getSpeed();
        for (int i = 0; i < twindexerLigaments.length; i++) {
            for (var ligament : twindexerLigaments[i]) {
                double angleChange = i == 0 ? -twindexerSpeed * 10 : twindexerSpeed * 10;
                ligament.setAngle(ligament.getAngle() + angleChange);
            }
        }

        for (var ligament : leftShooterLigaments) {            
            ligament.setAngle(ligament.getAngle() - robot.shooterLeft.getSetpoint() / 350.0);
        }

        for (var ligament : hoodLigaments) {            
            ligament.setAngle(robot.hood.getSetpoint());
        }
    }
}
