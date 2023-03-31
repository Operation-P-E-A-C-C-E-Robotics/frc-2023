package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;

import java.awt.*;
import java.util.Random;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

public class PhotonicHRI {
    AddressableLED led;
    AddressableLEDBuffer buffer;
    Random positionRandom = new Random();
    Random colorRandom = new Random();
    PhotonicLingualElement running;

    /**
     * the Photonic Human Robot Interface
     */
    public PhotonicHRI(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(buffer.getLength());
        led.start();
    }

    public void runElement(PhotonicLingualElement element) {
        if (running == element) return;
        if (running != null) {
            running.stop();
        }
        running = element;
        running.start();
    }

    public PhotonicLingualElement setSolidColor(int r, int g, int b) {
        return new PhotonicLingualElement(() -> {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, r, g, b);
            }
            led.setData(buffer);
        }, 0);
    }

    public final PhotonicLingualElement random = new PhotonicLingualElement(() -> {
            buffer.setRGB(
                    positionRandom.nextInt(buffer.getLength()),
                    colorRandom.nextInt(255),
                    colorRandom.nextInt(255),
                    colorRandom.nextInt(255)
            );

            led.setData(buffer);
        }, 0.02);

    int rainbowFirstPixelHue = 0;
    public final PhotonicLingualElement rainbow = new PhotonicLingualElement(() -> {
            for(var i = 0; i < buffer.getLength(); i++){
                final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
            led.setData(buffer);
        }, 0.03);

    public PhotonicLingualElement blink(int r, int g, int b, double period) {
        final boolean[] on = {false}; //not sure why this should be an array, but my editor says it should be
        return new PhotonicLingualElement(() -> {
            if (on[0]) {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, r, g, b);
                }
                led.setData(buffer);
            } else {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setRGB(i, 0,0,0);
                }
                led.setData(buffer);
            }
            on[0] = !on[0];
        }, period);
    }

    public PhotonicLingualElement fire(){
        //fancy moving flame pattern:
        //https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffect_Fire

        //AI: okay, heres how it works:
        //we have an array of integers called heat, which represents the heat of each pixel
        //we have a variable called cooling, which represents how much the heat decreases each frame
        //we have a variable called sparking, which represents how likely it is for a pixel to spark
        //we have a loop that runs 3 times
        //in each loop, we do the following:
        //for each pixel, we decrease the heat by the cooldown
        //for each pixel, we increase the heat by the average of the heat of the pixel to the left, the pixel to the right, and the pixel 2 to the right
        //for each pixel, we have a chance to increase the heat by a random amount
        //for each pixel, we set the color of the pixel to the color that corresponds to the heat of the pixel

        //SEAN: What will happen if we only run the loop a single time?
        //AI: It will look like a fire, but it will be very slow
        //SEAN: This function will be called every 20ms, so it will run 50 times a second. How fast is that?
        //AI: It's 20 times faster than the fire effect in the link
        //SEAN: so running it once should be okay?
        //AI: yes
        //SEAN: what is the best update time (in seconds) for this effect?
        //AI: 0.05
        //SEAN TO OTHER HUMANS: holy crap that link actually is a thing!
        
        //Logan: What in the fuck happened here?
        var heat = new int[buffer.getLength()];
        return new PhotonicLingualElement(() -> {
            for (var i = 0; i < buffer.getLength(); i++) {
                heat[i] = 0;
            }
            var cooling = 2;
            var sparking = 120;
            for (var i = 0; i < buffer.getLength(); i++) {
                // Step 1.  Cool down every cell a little
                heat[i] = Math.max(heat[i] - cooling, 0);
                // Step 2.  Heat from each cell drifts 'up' and diffuses a little
                heat[i] = (heat[i] + heat[(i + 1) % buffer.getLength()] + heat[(i + 2) % buffer.getLength()]) / 3;
                // Step 3.  Randomly ignite new 'sparks' near the bottom
                if (Math.random() * 255 < sparking) {
                    var y = Math.min(buffer.getLength() - 1, i + 2);
                    heat[y] = Math.min(255, (int) (heat[y] + 160 + Math.random() * 96));
                }
                // Step 4.  Map from heat cells to LED colors
                var color = heatToColor(heat[i]);
                buffer.setRGB(i, color[0], color[1], color[2]);
            }
            led.setData(buffer);
        }, 0.1);
    }

    private int[] heatToColor(int heat) {
        var red = 0;
        var green = 0;
        var blue = 0;
        if (heat < 85) {
            red = heat * 3;
            green = 255 - heat * 3;
            green = 0;
        } else {
            red = 255;
            green = 255 - (heat - 85) * 3;
        }
        return new int[]{red, green, blue};
    }

    public PhotonicLingualElement twinkle(int r, int g, int b) {
        return new PhotonicLingualElement(() -> {
            for (var i = 0; i < buffer.getLength(); i++) {
                if (Math.random() * 255 < 8) {
                    buffer.setRGB(i, r, g, b);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
            led.setData(buffer);
        }, 0.05);
    }

    public PhotonicLingualElement firev2(double cooling, int sparks, double sparking, int sparkHeight){
        var heat = new int[buffer.getLength()];
        return new PhotonicLingualElement(() -> {
            //cool each pixel a bit:
            for(var i = 0; i < buffer.getLength(); i++){
                heat[i] = Math.max(0, heat[i] - (int)(Math.random() * ((cooling * 10) / buffer.getLength() + 2)));
            }

            //ignite sparks:
            for(var i = 0; i < sparks; i++){
                if(Math.random() * 255 < sparking){
                    var y = buffer.getLength() - 1 - (int)(Math.random() * sparkHeight);
                    heat[y] += (int)(Math.random() * 160) + 95;
                }
            }

            //color:
            for(var i = 0; i < buffer.getLength(); i++){
                var color = heatToColorV2.interpolate(heat[i]);
                buffer.setLED(i, color);
            }

        }, 0.02);
    }

    ColorInterpolate heatToColorV2 = new ColorInterpolate(
            new int[]{0, 255},
            new int[]{255, 255},
            new int[]{255, 255},
            new int[]{0, 255}
    );



    public PhotonicLingualElement rainbowTwinkle(){
        final int[] rainbowTwinkleFirstPixelHue = {0};
        return new PhotonicLingualElement(() -> {
            for(var i = 0; i < buffer.getLength(); i++){
                final var hue = (rainbowTwinkleFirstPixelHue[0] + (i * 180 / buffer.getLength())) % 180;
                if (Math.random() * 255 < 8) {
                    buffer.setHSV(i, hue, 255, 128);
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
            rainbowTwinkleFirstPixelHue[0] += 3;
            rainbowTwinkleFirstPixelHue[0] %= 180;
            led.setData(buffer);
        }, 0.05);
    }

    public PhotonicLingualElement randomTwinkle(){
        return new PhotonicLingualElement(() -> {
            for(var i = 0; i < buffer.getLength(); i++){
                if (Math.random() * 255 < 8) {
                    buffer.setRGB(i, (int) (Math.random() * 255), (int) (Math.random() * 255), (int) (Math.random() * 255));
                } else {
                    buffer.setRGB(i, 0, 0, 0);
                }
            }
            led.setData(buffer);
        }, 0.05);
    }

    public void off() {
        if(running != null) running.stop();
        running = null;
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        led.setData(buffer);
    }

    public static class PhotonicLingualElement{
        private final Runnable update;
        private final Notifier notifier;
        private final double time;

        public PhotonicLingualElement(Runnable update, double time){
            this.update = update;
            this.time = time;

            if(time > 0) notifier = new Notifier(update);
            else notifier = null;
        }

        public void start(){
            if(notifier != null) notifier.startPeriodic(time);
            else update();
        }

        public void stop() {
            if(notifier != null) notifier.stop();
        }

        public void update() {
            update.run();
        }
    }

    public static class ColorInterpolate{
        int[] positions, hues, values, saturations;

        public ColorInterpolate(int[] hues, int[] values, int[] saturations, int[] positions){
            this.positions = positions;
            this.hues = hues;
            this.values = values;
            this.saturations = saturations;
        }

        public Color interpolate(int position){
            //AI! YOU HAVE NO INTERPOLATION FUNCTION! YOU HAVE TO WRITE IT FROM SCRATCH PLEASE!:
            for(var i = 0; i < positions.length; i++){
                if(positions[i] > position){
                    var hue = (int) (hues[i - 1] + (hues[i] - hues[i - 1]) * ((double) (position - positions[i - 1]) / (positions[i] - positions[i - 1])));
                    var value = (int) (values[i - 1] + (values[i] - values[i - 1]) * ((double) (position - positions[i - 1]) / (positions[i] - positions[i - 1])));
                    var saturation = (int) (saturations[i - 1] + (saturations[i] - saturations[i - 1]) * ((double) (position - positions[i - 1]) / (positions[i] - positions[i - 1])));
                    return Color.fromHSV(hue, saturation, value);
                }
            }
            return Color.fromHSV(hues[hues.length - 1], saturations[saturations.length - 1], values[values.length - 1]);
        }
    }
}
