package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import static edu.wpi.first.units.Units.Degrees;

public class IntakeCommands extends SubsystemBase {

    private final RobotContainer robot;

    public IntakeCommands(RobotContainer robot) {
        this.robot = robot;
    }

    public Command armDown() {
        return Commands.sequence(
                robot.arm.setAngle(Degrees.of(15)),
                robot.intake.outtakeDown(),
                Commands.waitUntil(() -> robot.arm.isDown()).withTimeout(2),
                robot.arm.set(-.2),
                robot.intake.stop())
                .withName("armDown");
    }

}