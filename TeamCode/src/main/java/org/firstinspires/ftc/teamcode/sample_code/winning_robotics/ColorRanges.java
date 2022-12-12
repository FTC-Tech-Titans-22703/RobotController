package org.firstinspires.ftc.teamcode.sample_code.winning_robotics;
import android.graphics.Color;
public class ColorRanges {

    public enum ColorFromHue {
        RED,
        GREEN,
        BLUE,
        INDETERMINATE
    }

    // Multiply the RGB values by a scale factor to amplify the measured values
    static final double SCALE_FACTOR = 255;

    public static ColorFromHue GetColor(int red, int green, int blue) {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (red * SCALE_FACTOR),
                (int) (green * SCALE_FACTOR),
                (int) (blue * SCALE_FACTOR),
                hsvValues);

        int hue = (int)hsvValues[0];
        // rough color ranges picked from the palette at http://www.workwithcolor.com
        if(hue >= 80 && hue <= 138) { // green
            return ColorFromHue.GREEN;
        }  else if((hue >= 344 && hue <= 360) || (hue >= 0 && hue <= 20)) { // RED
            return ColorFromHue.RED;
        }  else if(hue >= 180 && hue <= 270) { // BLUE
            return ColorFromHue.BLUE;
        }

        return ColorFromHue.INDETERMINATE;
    }
}