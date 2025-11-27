package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class TestMotor2Motor extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor testMotor0 = hardwareMap.get(DcMotor.class, "Test0");
        DcMotor testMotor1 = hardwareMap.get(DcMotor.class, "Test1");
        testMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        testMotor1.setPower(0);
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

            if(gamepad1.a){
                testMotor1.setPower(-0.7);
            }

            if (gamepad1.b){
                testMotor1.setPower(0);
            }

            testMotor0.setPower(speed);
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}
