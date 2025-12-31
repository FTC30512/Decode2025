package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ShooterTest", group = "Main")
public class Test_DcMotorEx extends LinearOpMode {

    // --- Servos ---
    private Servo gateServo, shooterServo;

    // --- Shooter Motor ---
    private DcMotor shooter;

    // --- Shooter Control ---
    private double shooterSpeed = 0.7;
    private boolean dpadUp = false;
    private boolean dpadDown = false;

    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();

        telemetry.addLine("Shooter TeleOp Ready. Press PLAY.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        while (opModeIsActive()) {

            // --- Trigger-based shooting ---
            if (gamepad1.right_trigger > 0.5) {
                shooter.setPower(0.85);
                sleep(250);
                shoot();
                telemetry.addLine("Shooting (0.85 power)");
            }

            if (gamepad1.left_trigger > 0.5) {
                shooter.setPower(1.0);
                sleep(250);
                shoot();
                telemetry.addLine("Shooting (1.0 power)");
            }

            // --- Adjust shooter speed ---
            if (gamepad1.dpad_up && !dpadUp) {
                shooterSpeed += 0.05;
            }
            if (gamepad1.dpad_down && !dpadDown) {
                shooterSpeed -= 0.05;
            }

            shooterSpeed = Math.max(0, Math.min(shooterSpeed, 1));
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            shooter.setPower(shooterSpeed);

            // --- Manual shoot button ---
            if (gamepad1.x) {
                shoot();
                telemetry.addLine("Manual Shot");
            } else {
                shooterServo.setPosition(0);
                gateServo.setPosition(0);
            }

            // --- Telemetry ---
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.addData("Shooter Speed Setting", shooterSpeed);
            telemetry.addData("Shooter Servo Position", shooterServo.getPosition());
            telemetry.update();
        }
    }

    // --- Hardware initialization ---
    private void initHardware() {
        shooter = hardwareMap.dcMotor.get("Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");

        shooterServo.setDirection(Servo.Direction.REVERSE);
        gateServo.setDirection(Servo.Direction.REVERSE);
    }

    // --- Shooting sequence ---
    private void shoot() {
        gateServo.setPosition(0.3);
        sleep(100);
        shooterServo.setPosition(0.35);
        sleep(250);
        shooterServo.setPosition(0);
        sleep(175);
        gateServo.setPosition(0);
        sleep(100);
    }
}