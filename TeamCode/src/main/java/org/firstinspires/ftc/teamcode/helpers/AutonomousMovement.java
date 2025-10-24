package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;


public class AutonomousMovement extends LinearOpMode{

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private IMU imu;
    private double yawOffset = 0;
    double wheelSize = 4.09;
    double ticksPerRev = 537.7;
    double countsPerInch;

    // robot's current spot
    double currentX = 0;
    double currentY = 0;
    double forwardFromRobot;
    double sidewaysFromRobot;

    public AutonomousMovement(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, Telemetry telemetry, HardwareMap hardwareMap, IMU imu) {
        this.leftFront = lf;
        this.leftRear = lr;
        this.rightFront = rf;
        this.rightRear = rr;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.imu = imu;
        countsPerInch = ticksPerRev / (wheelSize * Math.PI);
    }

    @Override
    public void runOpMode(){
        // setting up motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // math stuff for distance

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
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));
    }
    public void straightInches(double inches, double powerPct) {
        int ticks = (int) (inches * countsPerInch);
        powerPct /= 100;

        telemetry.addData("Ticks: ", ticks);
        telemetry.addData("powerPct", powerPct);

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

            telemetry.addData("Telemetry    ","Power:" + powerNow + "Fix: " + fix);
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

        currentX += inches;
        telemetry.addData("X Position", currentX);
        telemetry.addData("Y Position", currentY);
        telemetry.update();
        stopAndReset();
    }

    public void driveDiagonallyInches(double x, double y, double powerPct) {
        powerPct /= 100;
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

            leftFront.setPower(powerPct * lf - fix);
            leftRear.setPower(powerPct * lr - fix);
            rightFront.setPower(powerPct * rf + fix);
            rightRear.setPower(powerPct * rr + fix);
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

    public void goToPosition(double x, double y) {
        double fieldSize = 144;
        double boxSize = 40;
        double boxLeft = (fieldSize - boxSize) / 2;
        double boxRight = boxLeft + boxSize;
        double boxTop = (fieldSize - boxSize) / 2;
        double boxBottom = boxTop + boxSize;

        //does the path go through the box horizontally
        boolean hittingX = (currentX < boxRight && x > boxLeft) || (x < boxRight && currentX > boxLeft);
        //does the path go through the box vertically
        boolean hittingY = (currentY < boxBottom && y > boxTop) || (y < boxBottom && currentY > boxTop);

        //if it goes through the box
        if (hittingX && hittingY) {
            // go around the box
            if (currentY < boxTop) {
                straightInches(boxTop - currentY + 1, 30);
                currentY = boxTop + 1;
            } else if (currentY > boxBottom) {
                straightInches(-(currentY - boxBottom + 1), 30);
                currentY = boxBottom - 1;
            }

            if (currentX < boxLeft) {
                strafeInches("right", boxLeft - currentX + 1, 30);
                currentX = boxLeft + 1;
            } else if (currentX > boxRight) {
                strafeInches("left", currentX - boxRight + 1, 30);
                currentX = boxRight - 1;
            }
        }

        // final move
        double dx = x - currentX;
        double dy = y - currentY;
        driveDiagonallyInches(dx, dy, 30);
        telemetry.addData("X Position", currentX);
        telemetry.addData("Y Position", currentY);
        telemetry.update();
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
}
