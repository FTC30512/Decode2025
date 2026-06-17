package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Test2MotorJoystick extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx test1Motor = hardwareMap.get(DcMotorEx.class, "Motor1");
        DcMotorEx test2Motor = hardwareMap.get(DcMotorEx.class, "Motor2");
        test1Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        test2Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double toggle1 = 1;
        double toggle2 = 1;

        boolean rightTriggerPressed = false;
        boolean leftTriggerPressed = false;

        waitForStart();

        while (opModeIsActive()){
            double speed = -gamepad1.left_stick_y;

            if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
                toggle1 *= -1;
                toggle2 *= -1;
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger <= 0.5) {
                rightTriggerPressed = false;
            }

            if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
                toggle2 *= -1;
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger <= 0.5) {
                leftTriggerPressed = false;
            }

            test1Motor.setPower(speed * toggle1);
            test2Motor.setPower(speed * toggle2);
        }
    }
}
