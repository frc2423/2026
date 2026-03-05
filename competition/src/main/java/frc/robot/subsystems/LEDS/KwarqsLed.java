package frc.robot.subsystems.LEDS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
// import frc.robot.subsystems.swervedrive.Vision;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(18); // 18 on each side

    // private boolean isAutoScoring = false;
    // private boolean isEjectingPOOP = false;

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        return Alliance.Red.equals(DriverStation.getAlliance().get());
    }

    public KwarqsLed() {
        ledController.add("yellow", new Yellow());
        ledController.add("orange", new Orange());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("rainbow", new Rainbow());
        ledController.add("dark", new Dark());
        ledController.add("GreenCycle", new GreenCycle());
        ledController.add("RedCycle", new RedCycle());
        ledController.add("BlueCycle", new BlueCycle());
        ledController.add("POOP", new POOP());

        ledController.add("AutoDown", new AutoDown());
        ledController.set("dark");

        setDefaultCommand(disable());
    }

    public Command disable() {
        var command = run(() -> {
            // System.out.println("SEEES DARK!!!");

            ledController.set("dark");
        }).ignoringDisable(true);
        return command;
    }

    public Command setYellow() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("yellow");
        }).ignoringDisable(true);
        return command;
    }

     public Command setAutoDown() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("AutoDown");
        }).ignoringDisable(true);
        return command;
    }

    public Command setRainbow() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("rainbow");
        }).ignoringDisable(true);
        return command;
    }

    public Command setRedCycle() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("RedCycle");
        }).ignoringDisable(true);
        return command;
    }

    public Command setBlueCycle() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("BlueCycle");
        }).ignoringDisable(true);
        return command;
    }

    public Command setGreenCycle() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("GreenCycle");
        }).ignoringDisable(true);
        return command;
    }

    public Command setOrange() {
        var command = run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            ledController.set("yellow");
        }).ignoringDisable(true);
        // command.ignoringDisable(?true);
        return command;
    }

    public Command setPurple() {
        var command = run(() -> {
            ledController.set("purple");
        }).ignoringDisable(true);
        return command;
    }

    public Command setGreen() {
        var command = run(() -> {
            // System.out.println("!!!!!!!!");
            ledController.set("green");
        }).ignoringDisable(true);
        return command;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("led", () -> ledController.getCurrentLed(), null);
    }

}