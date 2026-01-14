package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.OldCode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;


public class BasicBlue1Auto extends LinearOpMode {

    // --- Servos ---
    private Servo gateServo, shooterServo;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake;
    private DcMotorEx shooter;
    // --- Sensors ---
    //private ColorSensor colorSensor;
    private IMU imu;
    double countsPerInch;
    private double shooterSpeed = 0.65;
    double wheelSize = 4.09;
    double ticksPerRev = 537.7;
    //HardwareMap hardwareMap;
    private double yawOffset = 0;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;

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
        initHardware();

        imu.resetYaw();

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
        gateServo.setPosition(0);
        shooterServo.setPosition(0);

        waitForStart();

        if (opModeIsActive()) {
            intake.setPower(1);
            shooter.setVelocity(2200);
            straightInches(-50,0.5);
            sleep(250);
            shoot();
            intake.setPower(-0.3);
            sleep(250);
            intake.setPower(1);
            sleep(250);
            shoot();
            sleep(250);
            shoot();
            pid_turn_by_gyro(-140, 0.4);
            strafeInches("right", 8, 0.5);
            straightInches(-36, 0.5);
            straightInches(36, 0.5);
            strafeInches("left", 8, 0.5);
            pid_turn_by_gyro(140, 0.4);
            sleep(250);
            shoot();
            intake.setPower(-0.3);
            sleep(250);
            intake.setPower(1);
            sleep(250);
            shoot();
            sleep(250);
            shoot();
            strafeInches("left", 20, 1);
            pid_turn_by_gyro(50, 0.5);
            intake.setPower(0);
            shooter.setPower(0);
            while(opModeIsActive()) {
                telemetry.addLine("Current Heading angle" + getHeading());
                telemetry.update();
            }
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
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
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
        shooter = hardwareMap.get(DcMotorEx.class,"Shooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //colorSensor = hardwareMap.colorSensor.get("colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
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
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

    }
    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
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

    public void straightInches(double inches, double powerPct) {
        int ticks = (int) (inches * countsPerInch);
        //powerPct /= 100;

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
            m.setPower(0.1);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double kP = 0.1;
        double minPower = 0.15;
        double angleFix = 0.02;
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

            telemetry.addData("startAngle", startAngle);
            telemetry.addData("leftFront.getCurrentPosition()", leftFront.getCurrentPosition());
            telemetry.addData("leftRear.getCurrentPosition()", leftRear.getCurrentPosition());
            telemetry.addData("rightFront.getCurrentPosition()", rightFront.getCurrentPosition());
            telemetry.addData("rightRear.getCurrentPosition()", rightRear.getCurrentPosition());
            telemetry.addLine("Power: " + powerNow + " Fix: " + fix + " Error: " + error);

            leftFront.setPower(powerNow);// - fix);
            leftRear.setPower(powerNow);// - fix);
            rightFront.setPower(powerNow);// + fix);
            rightRear.setPower(powerNow);// + fix);
            telemetry.update();
        }

        //currentY += inches;
        telemetry.addLine("Movement Done");
        //telemetry.addData("Y Position", currentY);
        //telemetry.update();
        stopAndReset();
    }
    public void strafeInches(String dir, double inches, double powerPct) {
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
        double minPower = 0.2;
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

            leftFront.setPower(powerNow);
            leftRear.setPower(-powerNow);
            rightFront.setPower(-powerNow);
            rightRear.setPower(powerNow);
        }

        //currentX += inches;
        //telemetry.addData("X Position", currentX);
        //telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
    }

    public void pid_turn_by_gyro(double targetYaw, double speed){
        double currentYaw = getHeading();
        double error;
        double actYaw = getHeading() + targetYaw;
        double kp = 0.01;
        while(opModeIsActive() && Math.abs(actYaw - getHeading()) > 0.1){

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
            //telemetry.addLine("Current Heading angle" + getHeading() + "Target Angle" + targetYaw + "Actual Target Yaw" + actTargetYaw);
            telemetry.update();
//            if(error > speed){
//                power = speed;
//            }
//            else if (error < -speed)
//            {
//                power = -speed;
//            }
//            else
//            {
//                power = error;
//            }
            setDrivePower(-power, -power, power, power );
        }
        setDrivePower(0,0,0,0 );

    }
    public void Turn_By_Gyro (double targetYaw, double leftPowerpct, double rightPowerpct){
        targetYaw = -targetYaw;
        double currentYaw = getHeading(); // Replace with IMU reading
        double actTargetYaw = currentYaw + targetYaw;
        //double error = targetYaw - currentYaw;
        //double correction = 0.5 * error;

        double kP = 0.05; // Optional: use for fine-tuned correction
        boolean turnClockwise = currentYaw < actTargetYaw;
        double error = actTargetYaw - currentYaw;

        double leftPower = leftPowerpct / 100.0;
        double rightPower = rightPowerpct/ 100.0;

        if (turnClockwise) {
            leftPower = -Math.abs(leftPower);
            rightPower = Math.abs(rightPower);
            while (opModeIsActive() && getHeading() < actTargetYaw) {
                telemetry.addLine("Current Heading angle" + getHeading());
                telemetry.addLine( "Target Angle" + targetYaw);
                telemetry.addLine("Actual Target Yaw" + actTargetYaw);
                //telemetry.addLine("Current Heading angle" + getHeading() + "Target Angle" + targetYaw + "Actual Target Yaw" + actTargetYaw);
                telemetry.update();
                setDrivePower(leftPower, leftPower, rightPower, rightPower);
            }
        } else {
            leftPower = Math.abs(leftPower);
            rightPower = -Math.abs(rightPower);
            while (opModeIsActive() && getHeading() > actTargetYaw) {
                telemetry.addLine("Current Heading angle" + getHeading());
                telemetry.addLine( "Target Angle" + targetYaw);
                telemetry.addLine("Actual Target Yaw" + actTargetYaw);
                telemetry.update();
                setDrivePower(leftPower, leftPower, rightPower, rightPower);
            }
        }

        setDrivePower(0.0, 0.0, 0.0, 0.0);
        telemetry.addLine("Current Heading angle" + getHeading());
        telemetry.addLine( "Target Angle" + targetYaw);
        telemetry.addLine("Actual Target Yaw" + actTargetYaw);
        telemetry.addData("Turn", "Completed to %.2f degrees", actTargetYaw);
        telemetry.update();
    }
    public void shoot() {
        gateServo.setPosition(0.3);
        sleep(100);
        shooterServo.setPosition(0.45);
        sleep(250);
        shooterServo.setPosition(0);
        sleep(175);
        gateServo.setPosition(0);
        sleep(100);
    }
}

