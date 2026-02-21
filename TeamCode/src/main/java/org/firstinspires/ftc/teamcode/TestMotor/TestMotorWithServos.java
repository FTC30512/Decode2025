package org.firstinspires.ftc.teamcode.TestMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestMotorWithServos extends LinearOpMode {
    private double shooterSpeedMax = 2450.0;
    private double shooterSpeedMin = 1000.0;
    private double shooterSpeed = shooterSpeedMax;
    private double shooterPower;
    boolean max = false;
    private double power = 0.0;
    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private long shooter_timer;
    private long steadystate_timer, rise_time;
    private boolean rise_time_set;
    private long timediff = 0;
    private boolean timediff_set, steadystate = false;
    private double[] scale = {10, 1, 0.1, 0.01};
    int index = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    /*
    @Override
    public void runOpMode(){
        initHardware();
        shooterSpeed = 2940;
        waitForStart();

        intake.setPower(1);
        shooterPower = 1;
        shooter.setPower(shooterPower);
        max = true;
        shooter_timer = System.currentTimeMillis();
        while (opModeIsActive()){
            if(gamepad1.dpadUpWasPressed())
            {
                shooterPower += 0.1;
            }
            if (gamepad1.dpadDownWasPressed())
            {
                shooterPower -= 0.1;
            }
            shooter.setPower(shooterPower);

            if (gamepad1.leftBumperWasPressed()){
                shoot();
            }

            dashboardTelemetry.addData("Target Velocity", shooterSpeed);
            dashboardTelemetry.addData("Shooter Velocity", shooter.getVelocity());
            dashboardTelemetry.update();
        }
    }
    */
    @Override
    public void runOpMode(){
        initHardware();
        //DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "Intake");
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
        //intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //Servo shooterServo = hardwareMap.servo.get("shooterServo");
        //Servo gateServo = hardwareMap.servo.get("gateServo");

        //gateServo.setDirection(Servo.Direction.REVERSE);
        //shooterServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        intake.setPower(1);
        shooterSpeed = shooterSpeedMax;
        shooter.setVelocity(shooterSpeed);
        max = true;
        shooter_timer = System.currentTimeMillis();
        while (opModeIsActive()){
            if(gamepad1.leftStickButtonWasPressed())
            {
                Kp = 1.0;
                Ki = 0.0;
                Kd = 0.0;
                Kf = 0.0;
                shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
            }
            if (gamepad1.rightBumperWasPressed())
            {
                index += 1;
            }
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                if (gamepad1.y) {
                    if (gamepad1.dpadUpWasPressed())
                        Kp += scale[index % scale.length];
                    if (gamepad1.dpadDownWasPressed())
                        Kp -= scale[index % scale.length];
                }
                if (gamepad1.b) {
                    if (gamepad1.dpadUpWasPressed())
                        Ki += 0.5;
                    if (gamepad1.dpadDownWasPressed())
                        Ki -= 0.5;
                }
                if (gamepad1.a) {
                    if (gamepad1.dpadUpWasPressed())
                        Kd += 0.5;
                    if (gamepad1.dpadDownWasPressed())
                        Kd -= 0.5;
                }
                if (gamepad1.x) {
                    if (gamepad1.dpadUpWasPressed())
                        Kf += scale[index % scale.length];
                    if (gamepad1.dpadDownWasPressed())
                        Kf -= scale[index % scale.length];
                }

                shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);
            }
            else
            {
                if(gamepad1.dpadUpWasPressed())
                {
                    shooterSpeed += 100.0;
                    shooter.setVelocity(shooterSpeed);
                }
                if (gamepad1.dpadDownWasPressed())
                {
                    shooterSpeed -= 100.0;
                    shooter.setVelocity(shooterSpeed);
                }
            }

//            if (gamepad1.leftBumperWasPressed())
//            {
//                if (max) {
//                    shooterSpeed = shooterSpeedMin;
//                    max = false;
//                }
//                else {
//                    shooterSpeed = shooterSpeedMax;
//                    max = true;
//                }
//                shooter_timer = System.currentTimeMillis();
//                timediff_set = false;
//                rise_time_set = false;
//                shooter.setVelocity(shooterSpeed);
//            }
            if (gamepad1.leftBumperWasPressed()){
                shoot();
                //shooterSpeed += 100.0;
                //shooter.setVelocity(-shooterSpeed);
            }
            //double vel_err = shooter.getVelocity() - 1420.0;
            //shooter.setPower(-0.0001 * vel_err);
            //shooter.setVelocity(1420.0);
            if(!timediff_set)
            {
                timediff = System.currentTimeMillis() - shooter_timer;
            }
            if (shooter.getVelocity() > shooterSpeed - 50 && shooter.getVelocity() < shooterSpeed + 50)
            {
                steadystate = true;
            }
            else
            {
                steadystate = false;
                steadystate_timer = System.currentTimeMillis();
            }
            if(steadystate && System.currentTimeMillis() - steadystate_timer >= 500) {
                timediff_set = true;
            }
/// /////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (shooter.getVelocity() > shooterSpeed - 50 && !rise_time_set)
            {
                rise_time_set = true;
                rise_time = System.currentTimeMillis() - shooter_timer;
            }


            PIDFCoefficients coeffs = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("Kp = " + coeffs.p);
            telemetry.addLine("Ki = " + coeffs.i);
            telemetry.addLine("Kd = " + coeffs.d);
            telemetry.addLine("Kf = " + coeffs.f);
            telemetry.addLine("Target Velocity = " + shooterSpeed);
            telemetry.addLine("Velocity = " + shooter.getVelocity());
            //telemetry.addLine("TIme Difference = " + timediff);
            telemetry.addLine("steady state flag = " + steadystate);
            telemetry.addLine("TIme to achieve steady state = " + (timediff - 500));
            telemetry.addLine("Rise time " + rise_time);
            dashboardTelemetry.addData("Target Velocity", shooterSpeed);
            dashboardTelemetry.addData("Shooter Velocity", shooter.getVelocity());
            dashboardTelemetry.update();
            telemetry.update();
        }
    }
    private void initHardware() {
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
    }

    public void shoot() {
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
        shooter_timer = System.currentTimeMillis();
        timediff_set = false;
        rise_time_set = false;
    }
}
