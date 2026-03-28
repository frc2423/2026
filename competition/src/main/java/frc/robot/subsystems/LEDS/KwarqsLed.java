package frc.robot.subsystems.LEDS;

import java.util.function.Supplier;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(18);

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
        ledController.add("YellowAndGreenCycle", new YellowAndGreenCycle());

        ledController.set("dark");

        setDefaultCommand(disable());
    }

    public Command setLeds(Supplier<String> name) {
        var command = run(() -> {
            ledController.set(name.get());
        }).withName(name.get()).ignoringDisable(true);
        return command;
    }

    public Command setLeds(String name) {
        return setLeds(() -> name);
    }

    public Command disable() {
        return setLeds("dark");
    }

    public Command setYellow() {
        return setLeds("yellow");
    }

    public Command YellowAndGreenCycle() {
        return setLeds("YellowAndGreenCycle");
    }

    public Command setAutoDown() {
        return setLeds("AutoDown");
    }

    public Command setRainbow() {
        return setLeds("rainbow");
    }

    public Command setRedCycle() {
        return setLeds("RedCycle");
    }

    public Command setBlueCycle() {
        return setLeds("BlueCycle");
    }

    public Command setGreenCycle() {
        return setLeds("GreenCycle");
    }

    public Command setOrange() {
        return setLeds("orange");
    }

    public Command setPurple() {
        return setLeds("purple");
    }

    public Command setGreen() {
        return setLeds("green");
    }

    @Logged
    public String getCurrentLed() {
        return ledController.getCurrentLed();
    }

    @Override
    public void periodic() {
        ledController.run();
    }
}