package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestMotorJoystick extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx testMotor = hardwareMap.get(DcMotorEx.class, "Test");
        testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            double speed = -gamepad1.left_stick_y;
            if (Math.abs(gamepad1.left_stick_y) <= 1) {
                testMotor.setPower(speed);
            }
            if (gamepad1.right_trigger>0.5){
                testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }
    }
}
