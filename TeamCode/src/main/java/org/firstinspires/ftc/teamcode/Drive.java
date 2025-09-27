package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;




public class Drive extends LinearOpMode{

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private double countsPerInch;
    private IMU imu;
    private double yawOffset = 0;

    // robot's current spot
    double currentX = 0;
    double currentY = 0;

    @Override
    public void runOpMode(){

        // math stuff for distance
        double wheelSize = 4.09;
        double ticksPerRev = 537.7;
        countsPerInch = ticksPerRev / (wheelSize * Math.PI);

        DcMotor[] motors = {leftFront, leftRear, rightFront, rightRear};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // reverse left motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup (for turning)
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        ));



    }
    public void straightInches(double inches, double power) {
        int ticks = (int) (inches * countsPerInch);
        power /= 100;

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

        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy())) {
            double avgTicks = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftRear.getCurrentPosition()) +
                    Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightRear.getCurrentPosition())) / 4.0;
            double inchesDone = avgTicks / countsPerInch;
            double leftToGo = Math.max(0.1, Math.abs(inches) - inchesDone);
            double powerNow = Math.max(minPower, Math.min(kP * leftToGo, power));

            double error = startAngle - getHeading();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double fix = error * angleFix;

            leftFront.setPower(powerNow - fix);
            leftRear.setPower(powerNow - fix);
            rightFront.setPower(powerNow + fix);
            rightRear.setPower(powerNow + fix);
        }

        currentY += inches;
        telemetry.addData("X Position", currentX);
        telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
    }

    public void strafeInches(String dir, double inches, double power) {
        power /= 100;
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
            double powerNow = Math.max(minPower, Math.min(kP * leftToGo, power));

            double error = startAngle - getHeading();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double fix = error * angleFix;

            leftFront.setPower(powerNow - fix);
            leftRear.setPower(-powerNow - fix);
            rightFront.setPower(-powerNow + fix);
            rightRear.setPower(powerNow + fix);
        }

        currentX += inches;
        telemetry.addData("X Position", currentX);
        telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
    }

    public void driveDiagonallyInches(double x, double y, double power) {
        power /= 100;
        double distance = Math.hypot(y, x);
        int ticks = (int) (distance * countsPerInch);

        double dx = x / distance;
        double dy = y / distance;

        double lf = dy + dx;
        double lr = dy - dx;
        double rf = dy - dx;
        double rr = dy + dx;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(lr), Math.max(Math.abs(rf), Math.abs(rr))));
        if (max > 1.0) {
            lf /= max;
            lr /= max;
            rf /= max;
            rr /= max;
        }

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) (ticks * lf));
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) (ticks * lr));
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) (ticks * rf));
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) (ticks * rr));

        for (DcMotor m : new DcMotor[]{leftFront, leftRear, rightFront, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double angleFix = 0.01;
        double startAngle = getHeading();

        while (opModeIsActive() && (leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy())) {
            double error = startAngle - getHeading();
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
            double fix = error * angleFix;

            leftFront.setPower(power * lf - fix);
            leftRear.setPower(power * lr - fix);
            rightFront.setPower(power * rf + fix);
            rightRear.setPower(power * rr + fix);
        }

        currentX += x;
        currentY += y;
        telemetry.addData("X Position", currentX);
        telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
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
}
