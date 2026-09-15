package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Speech {

    private static boolean disabled = false;

    public static void start() {
        disabled = false;
    }

    public static void stop() {
        disabled = true;
    }

    public static void say(String text) {
        if (!disabled) {
            NTHelper.setString("/robotSpeech", text);
        }
    }
}
