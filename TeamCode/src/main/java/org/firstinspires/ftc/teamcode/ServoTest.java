package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Ready — press left bumper to move servo.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                gateServo.setPosition(0.4);
                telemetry.addLine("Shooting");
            } else {
                gateServo.setPosition(0.1);
            }

            if (gamepad1.right_bumper){
                gateServo.setDirection(Servo.Direction.REVERSE);
            }

            telemetry.addData("Position", gateServo.getPosition());
            telemetry.update();
        }
    }
}
