package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.ColorSensor;
import android.graphics.Color;

import java.util.LinkedList;
import java.util.Queue;

public class DetectArtifacts {

    private final ColorSensor colorSensor;
    private final float[] hsvValues = new float[3];

    // For smoothing: store last few hue readings
    private final Queue<Float> hueHistory = new LinkedList<>();
    private static final int SMOOTH_WINDOW_SIZE = 5;  // average last 5 readings

    public DetectArtifacts(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public String getColor() {
        // Convert RGB to HSV
        Color.RGBToHSV(
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue(),
                hsvValues
        );

        float hue = hsvValues[0];
        int totalBrightness = colorSensor.red() + colorSensor.green() + colorSensor.blue();

        // === Smoothing ===
        hueHistory.add(hue);
        if (hueHistory.size() > SMOOTH_WINDOW_SIZE) {
            hueHistory.poll();  // keep window size constant
        }

        float avgHue = 0;
        for (float h : hueHistory) {
            avgHue += h;
        }
        avgHue /= hueHistory.size();

        // === Detection Logic ===

        // If lighting is too dark or hue is near background value (~174)
        if ((avgHue > 170 && avgHue < 180) || totalBrightness < 50) {
            return "Nothing";
        }

        if (avgHue > 120 && avgHue < 175) {
            return "Decode_green";
        } else if (avgHue > 200 && avgHue < 250) {
            return "Decode_purple";
        } else {
            return "Unknown";
        }
    }
}
