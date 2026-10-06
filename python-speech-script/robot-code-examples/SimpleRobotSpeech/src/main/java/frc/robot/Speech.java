package frc.robot;

public class Speech {
    public static void say(String text) {
        NTHelper.setString("/robotSpeech", text);
    }
}
