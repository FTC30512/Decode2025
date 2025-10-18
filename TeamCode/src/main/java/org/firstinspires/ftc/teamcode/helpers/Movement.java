package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Movement {
    private final DcMotor lf, lr, rf, rr;
    private final Gamepad gamepad;

    public Movement(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, Gamepad gamepad) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.gamepad = gamepad;
    }

    public void drive() {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        lf.setPower(fl);
        lr.setPower(bl);
        rf.setPower(fr);
        rr.setPower(br);
    }
}
