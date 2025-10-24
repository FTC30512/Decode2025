package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Movement {

    private final DcMotor lf, lr, rf, rr;
    private final Gamepad gamepad;
    private final boolean invertFL = false;
    private final boolean invertFR = false;
    private final boolean invertBL = false;
    private final boolean invertBR = false;

    public Movement(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, Gamepad gamepad) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.gamepad = gamepad;
    }

    public void drive() {


        // Raw sticks
        double y = -gamepad.left_stick_y; // forward is positive
        double x = gamepad.left_stick_x;  // right is positive
        double rx = gamepad.right_stick_x; // clockwise turn positive



        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double fl = y + x + rx; // front-left
        double fr = y - x - rx; // front-right
        double bl = y - x + rx; // back-left
        double br = y + x - rx; // back-right



        fl /= denominator;
        fr /= denominator;
        bl /= denominator;
        br /= denominator;

        // Apply per-motor inversion if needed (useful for testing)
        lf.setPower(fl);
        rf.setPower(fr);
        lr.setPower(bl);
        rr.setPower(br);
    }
}
