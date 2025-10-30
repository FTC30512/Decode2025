package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import android.graphics.Color;

@Disabled
public class Color_SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        float[] hsvValues = new float[3];

        telemetry.addLine(">>> Place object near color sensor <<<");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Convert RGB to HSV
            Color.RGBToHSV(
                    colorSensor.red(),
                    colorSensor.green(),
                    colorSensor.blue(),
                    hsvValues
            );

            float hue = hsvValues[0];
            String detectedColor = "Unknown";

            // (Temporary thresholds — we'll tune these based on your results)
            if (hue > 80 && hue < 160) {
                detectedColor = "Decode_green";
            } else if (hue > 260 && hue < 320) {
                detectedColor = "Decode_purple";
            }

            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Hue", "%.1f", hue);
            telemetry.addData("Detected", detectedColor);
            telemetry.update();

            sleep(100);
        }
    }
}
