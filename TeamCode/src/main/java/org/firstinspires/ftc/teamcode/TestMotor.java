package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class TestMotor extends LinearOpMode {
    @Override
    public void runOpMode(){
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor intake = hardwareMap.dcMotor.get("Intake");
        DcMotor shooterR = hardwareMap.dcMotor.get("ShooterR");
        DcMotor shooterL = hardwareMap.dcMotor.get("ShooterL");


        waitForStart();

        double speed = 0;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake.setPower(1);
                shooterR.setPower(speed);
                shooterL.setPower(-speed);
            }else {
                intake.setPower(0);
                shooterR.setPower(0);
                shooterL.setPower(0);
            }
            if (gamepad1.dpad_up){
                if (speed <= 0.9) {
                    speed += 0.05;
                    sleep(200);
                }
            }
            if (gamepad1.dpad_down){
                if (speed > 0) {
                    speed -= 0.05;
                    sleep(200);
                }
            }
            telemetry.addData("Speed: ", speed);
            telemetry.update();

        }
    }
}