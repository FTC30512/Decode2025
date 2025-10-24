package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo shooterServo = hardwareMap.servo.get("shooterServo");
        shooterServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Ready — press left bumper to move servo.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                shooterServo.setPosition(0.5);
                telemetry.addLine("Shooting");
            } else {
                shooterServo.setPosition(0);
            }

            telemetry.addData("Position", shooterServo.getPosition());
            telemetry.update();
        }
    }
}
