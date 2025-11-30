package org.firstinspires.ftc.teamcode.TestLED;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class TestLED {
    private LED redLED, greedLED;

    public void init(HardwareMap hwMap){
        redLED = hwMap.get(LED.class, "red_led");
        greedLED = hwMap.get(LED.class, "greed_led");
    }

    public void setRedLED(boolean isON){
        if (isON){
            redLED.on();
        }
        else {
            redLED.off();
        }
    }

    public void setGreedLED(boolean isON){
        if (isON){
            greedLED.on();
        }
        else {
            greedLED.off();
        }
    }
}
