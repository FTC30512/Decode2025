package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TeleOp.HelperTeleOp.Movement;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@TeleOp(name = "EW TeleOp", group = "Main")
public class EWTeleOp extends LinearOpMode {
    private LLResult llResult;

    // --- Servos ---
    private Servo gateServo, shooterServo;
    private Limelight3A limelight;

    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake, shooter;

    // --- Sensors ---
    private IMU imu;

    // --- Helper Classes ---
    private Movement movement;

    // --- Shooter Control ---
    private double shooterSpeed = 0.7;
    private boolean dpadUp = false;
    private boolean dpadDown = false;


    @Override
    public void runOpMode() {

        // --- Initialize hardware ---
        initHardware();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight.start();


        // --- Initialize helpers ---
        movement = new Movement(leftFront, leftRear, rightFront, rightRear, gamepad1, imu);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
        gamepad1.rumble(500);

        waitForStart();

        limelight.start();

        // --- Main loop ---
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
                telemetry.update();
            }

            if (gamepad1.left_bumper){
                intake.setPower(-1);
            }else {
                intake.setPower(1);
            }

            // --- Driving ---
            movement.drive();

            // --- Intake ---
            intake.setPower(1);

            // ---Shooter---
            shooter.setPower(1);

            // --- Shooting logic ---
            if (gamepad1.right_trigger > 0.5){
                shooter.setPower(0.85);
                while (shooter.getPower() != 0.85){

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
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // --- Shooting method ---
    public void shoot() {
        pid_turn_by_gyro(llResult.getTx(), 0.5);
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

    private double getHeading() {
        double angle = getRawHeading();
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public void pid_turn_by_gyro(double targetYaw, double speed){
        targetYaw=-targetYaw;
        double currentYaw = getHeading();
        double error;
        double actYaw = getHeading() + targetYaw;
        double kp = 0.01;
        while(Math.abs(actYaw - getHeading()) > 0.85){

            error = kp * (actYaw - getHeading());
            double power;
            if(error > 0)
                power = Math.min(Math.max(error, 0.1), speed);
            else
                power = Math.min(Math.max(error, -speed), -0.1);

            telemetry.addLine("Current Heading angle" + getHeading());
            telemetry.addLine( "Target Angle" + targetYaw);
            telemetry.addLine("Actual Target Yaw" + actYaw);
            telemetry.addLine("Actual Power " + power);
            telemetry.update();
            setDrivePower(-power, -power, power, power );
        }
        setDrivePower(0,0,0,0 );

    }

    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }
}
