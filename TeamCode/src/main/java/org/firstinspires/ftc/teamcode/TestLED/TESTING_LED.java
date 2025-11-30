package org.firstinspires.ftc.teamcode.TestLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TESTING_LED extends OpMode {
    TestLED led = new TestLED();

    @Override
    public void init() {
        led.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            led.setRedLED(true);
            led.setGreedLED(false);
        }else if (gamepad1.b){
            led.setGreedLED(true);
            led.setRedLED(false);
        }else if (gamepad1.y){
            led.setRedLED(true);
            led.setGreedLED(true);
        }
    }
}
