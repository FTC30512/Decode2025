package org.firstinspires.ftc.teamcode.shooting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.Movement;
import org.firstinspires.ftc.teamcode.helpers.PointToAprilTag;
import org.firstinspires.ftc.teamcode.helpers.Velocity;

public class Shoot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }


    ColorSensor colorSensor;
    DcMotor lf, lr, rf, rr, shooter;
    Servo sS;

    public Shoot(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr, DcMotor shooter, Servo sS, ColorSensor cS, Telemetry telemetry) {
        this.lf = lf;
        this.lr = lr;
        this.rf = rf;
        this.rr = rr;
        this.sS = sS;
        this.telemetry = telemetry;
        this.shooter = shooter;
        this.colorSensor = cS;
    }

    PointToAprilTag pointToAprilTag = new PointToAprilTag(hardwareMap, telemetry);
    Velocity velocity = new Velocity(pointToAprilTag);



    boolean readyToShoot = false;

    public void servoShoot() {
        if ((colorSensor.green() > 0.5 && colorSensor.blue() < 0.3 && colorSensor.red() < 0.3) || (colorSensor.blue() > 0.3 && colorSensor.red() > 0.3 && colorSensor.green() < 0.4)){
            sS.setPosition(0.5);
            telemetry.addLine("Shooting");
            readyToShoot = false;
        } else {
            sS.setPosition(0);
        }

    }
    public void getReady(){
        // --- Update AprilTag each loop ---
        pointToAprilTag.updateTagDistance();

        // --- Turn toward tag if right trigger is pressed ---
        if (gamepad1.right_trigger > 0.5) {
            pointToAprilTag.turnToTag(lf, lr, rf, rr);
            shooter.setPower(velocity.getSpeed(hardwareMap, telemetry));
            readyToShoot = true;
        }
    }
}
