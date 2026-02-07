package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonomousImplements {

    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Limelight3A limelight;

    public void init(AutonomousConstants constants, HardwareMap hardwareMap){
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocityPIDFCoefficients(constants.Kp, constants.Ki, constants.Kd, constants.Kf);
        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
        gateServo.setPosition(0);
        shooterServo.setPosition(0);
    }
    public void stopall(){
        intake.setPower(0);
        shooter.setPower(0);
        gateServo.setPosition(0);
        shooterServo.setPosition(0);
    }
    public void setIntakePower(double power){
        intake.setPower(power);
    }
    public void setShooterVelocity(double velocity){
        shooter.setVelocity(velocity);
    }
    public double getShooterVelocity(){
        return shooter.getVelocity();
    }
    public void setLimelightPipeline(int index){
        limelight.pipelineSwitch(index);
    }
    public LLResult getLimelightResult(){
        return limelight.getLatestResult();
    }
    public void updatelimelightOrientation(double orientation){
        limelight.updateRobotOrientation(orientation);
    }
    public void shoot() {
        intake.setPower(0);
        shooterServo.setPosition(0.35);
        sleep(250);
        shooterServo.setPosition(0);
        sleep(250);
        intake.setPower(-0.25);
        sleep(100);
        intake.setPower(1);
    }


}
