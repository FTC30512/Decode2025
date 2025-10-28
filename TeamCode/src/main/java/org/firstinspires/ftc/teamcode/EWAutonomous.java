package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.helpers.AutonomousMovement;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Autonomous(name = "EW Autonomous", group = "Autonomous")
public class EWAutonomous extends LinearOpMode {

    // --- Servos ---
    private Servo gateServo, shooterServo;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake, shooter;
    // --- Sensors ---
    private ColorSensor colorSensor;
    private IMU imu;
    double countsPerInch;
    double wheelSize = 4.09;
    double ticksPerRev = 537.7;
    //HardwareMap hardwareMap;
    private double yawOffset = 0;

    /*public EWAutonomous() {

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
        countsPerInch = ticksPerRev / (wheelSize * Math.PI);
    }*/
    @Override
    public void runOpMode() throws InterruptedException {

        initHardwareTest();

        /*
        countsPerInch = ticksPerRev / (wheelSize * Math.PI);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        */
        countsPerInch = ticksPerRev / (wheelSize * Math.PI);

        //AutonomousMovement auto = new AutonomousMovement(leftFront, leftRear, rightFront, rightRear, telemetry, hardwareMap, imu);

        telemetry.addLine("Initialized - Ready to Run");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            straightInches(100, 10); // move forward 10 inches
            sleep(500);
            straightInches(-90, 60); // move backward 10 inches
            strafeInches("left", 50, 10);
            strafeInches("right", 20, 90);
        }
    }

    // --- Hardware initialization ---
    private void initHardwareTest() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);
    }

    // --- Hardware initialization ---
    private void initHardware() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.dcMotor.get("Shooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        // AprilTag Library (optional here)
        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();
    }

    private void stopAndReset() {
        for (DcMotor motor : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sleep(100);
    }

    private double getHeading() {
        double angle = getRawHeading() - yawOffset;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
    public double getYaw() {
        //double angles;
        ///angles = imu.getRobotYawPitchRollAngles();
        //double yaw = angles; // Yaw is typically the first angle returned
        // Define hub orientation on the robot
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        // Initialize IMU from hardware
        // Initialize IMU with orientation
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        imu.initialize(parameters);


        // Get yaw, pitch, roll angles
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Yaw", yaw);
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");

        return yaw;
    }

    public void straightInches(double inches, double powerPct) {
        int ticks = (int) (inches * countsPerInch);
        powerPct /= 100;

        telemetry.addData("Ticks: ", ticks);
        telemetry.addData("powerPct", powerPct);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + ticks);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double kP = 0.1;
        double minPower = 0.15;
        double angleFix = 0.01;
        double startAngle = getHeading();

        telemetry.addData("startAngle", startAngle);
        telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
        telemetry.addData("leftRear.getCurrentPosition()", leftRear.getCurrentPosition());
        telemetry.addData("rightFront.getCurrentPosition()", rightFront.getCurrentPosition());
        telemetry.addData("rightRear.getCurrentPosition()", rightRear.getCurrentPosition());
        telemetry.update();
        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy())) {

            double avgTicks = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftRear.getCurrentPosition()) +
                    Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0;
            double inchesDone = avgTicks / countsPerInch;
            double leftToGo = Math.max(0.1, Math.abs(inches) - inchesDone);
            double powerNow = Math.max(minPower, Math.min(kP * leftToGo, powerPct));

            double error = startAngle - getHeading();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double fix = error * angleFix;

            telemetry.addLine("Power: " + powerNow + " Fix: " + fix);
            leftFront.setPower(powerNow - fix);
            leftRear.setPower(powerNow - fix);
            rightFront.setPower(powerNow + fix);
            rightRear.setPower(powerNow + fix);
            //telemetry.update();
        }

        //currentY += inches;
        telemetry.addLine("Movement Done");
        //telemetry.addData("Y Position", currentY);
        //telemetry.update();
        stopAndReset();
    }
    public void strafeInches(String dir, double inches, double powerPct) {
        powerPct /= 100;
        double fixFactor = 1.07;
        int ticks = (int) (inches * fixFactor * countsPerInch);

        if (dir.equals("left")) {
            ticks *= -1;
        }


        leftFront.setTargetPosition(leftFront.getCurrentPosition() + ticks);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - ticks);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + ticks);

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double kP = 0.1;
        double minPower = 0.15;
        double angleFix = 0.01;
        double startAngle = getHeading();

        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy())) {
            double avgTicks = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftRear.getCurrentPosition()) +
                    Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0;
            double inchesDone = avgTicks / countsPerInch;
            double leftToGo = Math.max(0.1, Math.abs(inches) - inchesDone);
            double powerNow = Math.max(minPower, Math.min(kP * leftToGo, powerPct));

            double error = startAngle - getHeading();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double fix = error * angleFix;

            leftFront.setPower(powerNow - fix);
            leftRear.setPower(-powerNow - fix);
            rightFront.setPower(-powerNow + fix);
            rightRear.setPower(powerNow + fix);
        }

        //currentX += inches;
        //telemetry.addData("X Position", currentX);
        //telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
    }

    public void Turn_By_Gyro (double targetYaw, double leftPowerpct, double rightPowerpct){
        double currentYaw = getYaw(); // Replace with IMU reading
        double error = targetYaw - currentYaw;
        double correction = 0.5 * error;

        double kP = 0.05; // Optional: use for fine-tuned correction
        boolean turnClockwise = currentYaw < targetYaw;

        double leftPower = leftPowerpct / 100.0;
        double rightPower = rightPowerpct/ 100.0;

        if (turnClockwise) {
            leftPower = Math.abs(leftPower);
            rightPower = -Math.abs(rightPower);
            while (getYaw() < targetYaw) {
                setDrivePower(leftPower, leftPower, rightPower, rightPower);
            }
        } else {
            leftPower = -Math.abs(leftPower);
            rightPower = Math.abs(rightPower);
            while (getYaw() > targetYaw) {
                setDrivePower(leftPower, leftPower, rightPower, rightPower);
            }
        }

        telemetry.addData("Turn", "Completed to %.2f degrees", targetYaw);
        telemetry.update();
    }
}
