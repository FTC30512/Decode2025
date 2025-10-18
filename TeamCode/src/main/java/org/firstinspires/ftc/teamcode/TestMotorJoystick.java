package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestMotorJoystick extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx testMotor = hardwareMap.get(DcMotorEx.class, "Test");
        testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            double speed = -gamepad1.left_stick_y;
            testMotor.setPower(speed);
        }
    }
}
