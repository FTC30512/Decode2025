package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestMotorBumper extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor testMotor = hardwareMap.get(DcMotor.class, "Test");
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        double speed = 0;

        while (opModeIsActive()){
            speed = Math.max(0.3, Math.min(speed,1));

            if(gamepad1.left_bumper){
                if (speed>0){
                    speed-=0.1;
                    sleep(200);
                }
            }
            if (gamepad1.right_bumper){
                if (speed<=0.9){
                    speed+=0.1;
                    sleep(200);
                }
            }
            testMotor.setPower(speed);
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}
