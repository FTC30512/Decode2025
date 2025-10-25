package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Movement {

    private final DcMotor lf, lr, rf, rr;
    private final Gamepad gamepad;
    private final IMU imu;

    public Movement(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, Gamepad gamepad, IMU imu) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.gamepad = gamepad;
        this.imu = imu;
    }

    public void drive() {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        // Reset heading button (options)
        if (gamepad.options) {
            imu.resetYaw();
        }

        double offset = 0; // adjust until forward is truly forward
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + offset;

        // Field-centric rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//        rotX = rotX * 1.1; // optional strafe compensation

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        lf.setPower(frontLeftPower);
        lr.setPower(backLeftPower);
        rf.setPower(frontRightPower);
        rr.setPower(backRightPower);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}
