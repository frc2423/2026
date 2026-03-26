package frc.robot.subsystems.LEDS;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class YellowAndGreenCycle implements Led {
    private double phase = 0.0;

    // Controls how many wave cycles appear across the strip
    private static final double WAVES_ON_STRIP = 1.0;

    // Controls animation speed
    private static final double SPEED = 0.2;

    public void start(AddressableLEDBuffer buffer, int length) {
        phase = 0.0;
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        int stripLength = buffer.getLength();

        for (int i = 0; i < stripLength; i++) {
           
            double x = (2.0 * Math.PI * WAVES_ON_STRIP * i / stripLength) + phase;

           
            double greenWave = Math.sin(x);

            
            double yellowWave = Math.cos(x - (3.0 * Math.PI / 2.0));

           
            int greenBrightness = (int) (Math.max(0, greenWave) * 255);
            int yellowBrightness = (int) (Math.max(0, yellowWave) * 255);

          
            int red = yellowBrightness;
            int green = Math.min(255, greenBrightness + yellowBrightness);
            int blue = 0;

            buffer.setRGB(i, red, green, blue);
        }

       
        phase += SPEED;
    }

    public void end(AddressableLEDBuffer buffer, int length) {
    }
}