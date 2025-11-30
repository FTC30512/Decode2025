package org.firstinspires.ftc.teamcode.TestMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestMotorWithServos extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "Intake");
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo shooterServo = hardwareMap.servo.get("shooterServo");
        Servo gateServo = hardwareMap.servo.get("gateServo");

        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double speed = gamepad1.left_stick_y;
            shooter.setPower(-1);
            intake.setPower(speed);
            if (gamepad1.right_trigger>0.5) {
                gateServo.setPosition(0.3);
                sleep(100);
                shooterServo.setPosition(0.35);
                sleep(250);
                shooterServo.setPosition(0);
                sleep(175);
                gateServo.setPosition(0);
                sleep(10);
            } else {
                shooterServo.setPosition(0);
            }

            if (gamepad1.right_bumper){
                shooterServo.setDirection(Servo.Direction.REVERSE);
            }
        }
    }
}
