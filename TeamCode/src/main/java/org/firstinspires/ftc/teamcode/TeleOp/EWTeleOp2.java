package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.TeleOp.HelperTeleOp.Movement;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@TeleOp(name = "EW TeleOp 2", group = "Main")
public class EWTeleOp2 extends LinearOpMode {

    // --- Servos ---
    private Servo gateServo, shooterServo;

    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake;
    private DcMotorEx shooter;

    // --- Sensors ---
    private IMU imu;

    // --- Helper Classes ---
    private Movement movement;

    // --- Shooter Control ---
    //private double shooterSpeed = 0.7;
    private boolean dpadUp = false;
    private boolean dpadDown = false;
    private int shooterSpeedMin = 2200, shooterSpeedMax = 2475;
    private double shooterSpeed;

    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;


    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();

        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, gamepad1, imu);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- Main loop ---
        while (opModeIsActive()) {

            if (gamepad1.left_bumper){
                intake.setPower(-1);
            }else {
                intake.setPower(1);
            }

            // --- Driving ---
            movement.drive();

            // --- Intake ---
            intake.setPower(1);

            // --- Shooting logic ---
            if (gamepad1.right_trigger > 0.5){
                shooterSpeed = shooterSpeedMin;
                sleep(200);
                shoot();
                telemetry.addLine("Shooting");
            }
            if (gamepad1.left_trigger > 0.5){
                shooterSpeed = shooterSpeedMax;
                sleep(400);
                shoot();
                telemetry.addLine("Shooting");
            }
            // ---Shooter---

            shooter.setVelocity(shooterSpeed);

            // --- Shooting logic ---
            if (gamepad1.right_trigger > 0.5){
                shooter.setVelocity(shooterSpeedMin);
                sleep(250);
                shoot();
                telemetry.addLine("Shooting");
            }
            if (gamepad1.left_trigger > 0.5){
                shooter.setVelocity(shooterSpeedMax);
                while (shooter.getPower() != 1){

                }
                sleep(250);
                shoot();
                telemetry.addLine("Shooting");
            }

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

            if (gamepad1.x) {
                shoot();
                telemetry.addLine("Shooting");
            } else {
                shooterServo.setPosition(0);
                gateServo.setPosition(0);
            }

            // --- Telemetry ---
            telemetry.addData("Shooter Servo Position", shooterServo.getPosition());
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.update();
        }
    }

    // --- Hardware initialization method ---
    private void initHardware() {
        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");

        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        // Sensors
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);
        shooterSpeed = shooterSpeedMax;

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();
    }

    // --- Shooting method ---
    public void shoot() {
        intake.setPower(0);
        gateServo.setPosition(0.3);
        sleep(100);
        shooterServo.setPosition(0.35);
        sleep(250);
        shooterServo.setPosition(0);
        sleep(175);
        gateServo.setPosition(0);
        sleep(100);
        intake.setPower(1);
    }

//    public void shoot() {
//        gateServo.setPosition(0.3);
//        sleep(100);
//        shooterServo.setPosition(0.45);
//        sleep(250);
//        shooterServo.setPosition(0);
//        sleep(175);
//        gateServo.setPosition(0);
//        sleep(100);
//    }
}
