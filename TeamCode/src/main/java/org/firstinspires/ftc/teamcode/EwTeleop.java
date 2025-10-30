package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.DetectArtifacts;
import org.firstinspires.ftc.teamcode.helpers.Movement;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@TeleOp(name = "EW TeleOp", group = "Main")
public class EWTeleOp extends LinearOpMode {

    // --- Servos ---
    private Servo gateServo, shooterServo;

    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake, shooter;

    // --- Sensors ---
    private ColorSensor colorSensor;
    private IMU imu;

    // --- Helper Classes ---
    private Movement movement;
    private DetectArtifacts detectArtifacts;

    // --- Shooter Control ---
    private double shooterSpeed = 0.7;
    private boolean dpadUp = false;
    private boolean dpadDown = false;

    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();

        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, gamepad1, imu);
        detectArtifacts = new DetectArtifacts(colorSensor);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        shooter.setPower(shooterSpeed);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- Main loop ---
        while (opModeIsActive()) {

            // --- Adjust shooter speed with bumpers/triggers (edge detection) ---
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

            // --- Driving ---
            movement.drive();

            // --- Intake ---
            intake.setPower(1);

            // --- Shooting logic ---
            if (gamepad1.right_trigger > 0.5){
                shooter.setPower(0.7);
                while (shooter.getPower() != 0.7){

                }
                sleep(250);
                shoot();
                telemetry.addLine("Shooting");
            }
            if (gamepad1.left_trigger > 0.5){
                shooter.setPower(1);
                while (shooter.getPower() != 1){

                }
                sleep(250);
                shoot();
                telemetry.addLine("Shooting");
            }

            String detected = detectArtifacts.getColor();
            if (gamepad1.right_bumper) {
                shoot();
                telemetry.addLine("Shooting");
            } else {
                shooterServo.setPosition(0.65);
                gateServo.setPosition(0);
            }

            // --- Telemetry ---
            telemetry.addData("Detected Color", detected);
            telemetry.addData("Shooter Servo Position", shooterServo.getPosition());
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.addData("Shooter Speed Setting", shooterSpeed);
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
        shooter = hardwareMap.dcMotor.get("Shooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        // Sensors
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();
    }

    // --- Shooting method ---
    public void shoot() {
        gateServo.setPosition(0.3);
        sleep(200);
        shooterServo.setPosition(1);
        sleep(500);
        shooterServo.setPosition(0.65);
        sleep(350);
        gateServo.setPosition(0);
        sleep(200);
    }
}
