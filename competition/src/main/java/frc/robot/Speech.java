package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import frc.robot.utils.HubTracker;

public class Speech {

    private static boolean disabled = false;
    private static String phrases[] = { "duck", "Exterminate", "Destroy", "Eradicate", "Shoot", "fire", "attack", "go",
            "onwards", "fire in the hole", "terminator-2-hasta-lavista-baby.mp3", "Duck and cover", "R2D2_beep.mp3",
            "exterminate-short.mp3" };
    private static int randomNumber;
    private static String endings[] = { "ill-be-back-arnold-schwarzenegger-the-terminator.mp3", "Bye Bye",
    "See you later, alligator", "gg no re", "good game",
    "The first law of robotics is: A robot may not injure a human being or, through inaction, allow a human being to come to harm.",
    "The second law of robotics is: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.",
    "The second law of robotics is: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.",
    "The zero-ith law of robotics is: A robot may not harm humanity, or, by inaction, allow humanity to come to harm.",
    "outro_song.mp3",
    "The first law of amory: We don't talk about Amory",
    "sad_trombone.mp3",
    "aughaughagugh.mp3" };
    
    boolean areAtSetpoint;
    private RobotContainer robotContainer;

    public Speech(RobotContainer container) {
        robotContainer = container;
    }

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

    public static void sayRandomPhrase() {
        int oldNumber = randomNumber;
        randomNumber = (int) (Math.random() * phrases.length);
        while (oldNumber == randomNumber) {
            randomNumber = (int) (Math.random() * phrases.length);
        }
        Speech.say(phrases[randomNumber]);
    }

    public void periodic() {
        sayPhraseWhenShooterIsRevved();
        countdownShift();
    }

    private void sayPhraseWhenShooterIsRevved() {
        boolean wereAtSetpoint = areAtSetpoint;
        areAtSetpoint = Math
                .abs(robotContainer.shooterRight.getVelocity() - robotContainer.shooterRight.getSetpoint()) <= 300
                && Math.abs(
                        robotContainer.shooterLeft.getVelocity() - robotContainer.shooterLeft.getSetpoint()) <= 300
                && robotContainer.shooterRight.getSetpoint() != 0 && robotContainer.shooterLeft.getSetpoint() != 0;
        if (areAtSetpoint && !wereAtSetpoint) {
            sayRandomPhrase();
        }
    }

    private void countdownShift() {
        if (160 - HubTracker.getMatchTime() < 15) {
            Speech.say((Integer.toString((int) (HubTracker.timeRemainingInCurrentShift().in(Seconds)) - 1)));
        }
        if (HubTracker.getMatchTime() < 5) {
            Speech.start();
        }
        // System.out.println(HubTracker.timeRemainingInCurrentShift().in(Seconds));
        if (HubTracker.timeRemainingInCurrentShift().in(Seconds) < 1) {
            // System.out.println("I'm working!");
            String shift = HubTracker.getNextShift().toString();
            if (shift.contains("_")) {
                int index = shift.indexOf("_");
                shift = shift.substring(0, index) + " " + shift.substring(index + 1);
            }
            if (shift == "AUTO") {
                int randomIndex = (int) (Math.random() * endings.length);
                Speech.say(endings[randomIndex]);
                Speech.stop();
            } else {
                Speech.say("Starting " + shift);
            }
        }
    }

}
