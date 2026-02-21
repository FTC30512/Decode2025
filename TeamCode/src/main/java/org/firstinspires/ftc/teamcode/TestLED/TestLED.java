package org.firstinspires.ftc.teamcode.TestLED;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "BlinkinColorExample", group = "Robot")
public class TestLED extends LinearOpMode {

    private RevBlinkinLedDriver blinkinLedDriver;

    @Override
    public void runOpMode() {
        // Initialize the Blinkin Driver from the hardware map configuration name
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Change color to green when 'a' is pressed
            if (gamepad1.a) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            // Change color to red when 'b' is pressed
            else if (gamepad1.b) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            // Change color to blue when 'x' is pressed
            else if (gamepad1.x) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            // Default to a different color (e.g., solid white) if no buttons are pressed
            else {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
        }
    }
}
