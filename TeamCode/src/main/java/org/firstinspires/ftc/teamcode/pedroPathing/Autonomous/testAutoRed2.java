package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static android.os.SystemClock.sleep;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Autonomous(name = "AutoRed2", group = "Autonomous")
public class testAutoRed2 extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(testAutoRed2.class);

    //public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private LLResult llResult;

    private final Pose startPose = new Pose(86.500, 11.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(84.000, 19.000, Math.toRadians(64));
    private final Pose firstRowStartPose = new Pose(98.000, 60.000, Math.toRadians(180));
    private final Pose firstRowEndPose = new Pose(120.000, 60, Math.toRadians(180));
    private final Pose secondRowStartPose = new Pose(98.000, 60.000, Math.toRadians(180));
    private final Pose secondRowEndPose = new Pose(120.000, 60.000, Math.toRadians(180));
    private final Pose thirdRowStartPose = new Pose(98.000, 40.000, Math.toRadians(180));
    private final Pose thirdRowEndPose = new Pose(120.000, 40.000, Math.toRadians(180));
    private final Pose endPose = new Pose(105.32586660900631, 33.079429420265015, Math.toRadians(0));
    double collectSpeed = 0.3;
    private enum PathState{
        STARTPOS_SHOOTPOS,
        SHOOTPOS_FIRSTROW,
        SHOOTPOS_SECONDROW,
        SHOOTPOS_THIRDROW,
        COLLECT_FIRSTROW,
        COLLECT_SECONDROW,
        COLLECT_THIRDROW,
        FIRSTROW_SHOOTPOS,
        SECONDROW_SHOOTPOS,
        THIRDROW_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT,
        STOP

    }
    PathState pathState;
    public PathChain pathStarttoShoot, pathShoottoSecond, pathSecondCollect, pathSecondtoShoot, pathShoottoThird, pathThirdCollect, pathThirdtoShoot, pathShoottoEnd;
    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Limelight3A limelight;
    private int shooterSpeed = 2475;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private YawPitchRollAngles orientation;
    //private double Kp = 25.0, Ki = 3.0, Kd = 0.0, Kf = 2.8;
    private IMU imu;
    boolean belly, firstrow, secondrow, thirdrow = false;



    @Override
    public void init() {
        initHardware();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1.0);

        buildPaths();
        pathState = PathState.STARTPOS_SHOOTPOS;

        imu.resetYaw();
        belly = true;
        gateServo.setPosition(0);
        shooterServo.setPosition(0);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // --- Hardware initialization ---
    private void initHardware() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");


        intake = hardwareMap.dcMotor.get("Intake");
            shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.start();
    }

    @Override
    public void start() {
        pathState = PathState.STARTPOS_SHOOTPOS;
        intake.setPower(1);
        shooter.setVelocity(shooterSpeed);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = limelight.getLatestResult();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Max Power", follower.getMaxPowerScaling());
        telemetry.update();
    }

    public void buildPaths() {

        pathStarttoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, shootPose)
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        //Wait5 = 2000;

        pathShoottoSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, secondRowStartPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowStartPose.getHeading())
                .build();

        //Wait6 = 2000;

        pathSecondCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(secondRowStartPose, secondRowEndPose)
                )
                .setLinearHeadingInterpolation(secondRowStartPose.getHeading(), secondRowEndPose.getHeading())
                .build();

        pathSecondtoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                secondRowEndPose,
                                new Pose(84.40694769008739, 63.48483482567039),
                                shootPose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(secondRowEndPose.getHeading(), shootPose.getHeading())
                .build();

        pathShoottoThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, thirdRowStartPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowStartPose.getHeading())
                .build();

        pathThirdCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(thirdRowStartPose, thirdRowEndPose)
                )
                .setLinearHeadingInterpolation(thirdRowStartPose.getHeading(), thirdRowEndPose.getHeading())
                .build();

        pathThirdtoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(thirdRowEndPose, shootPose)
                )
                .setLinearHeadingInterpolation(thirdRowEndPose.getHeading(), shootPose.getHeading())
                .setHeadingConstraint(0.0001)
                .build();

        pathShoottoEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, endPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }
    long waitStart = 0;
    boolean waiting = false;

    public void stopall() {
        intake.setPower(0);
        shooter.setPower(0);
        gateServo.setPosition(0);
        shooterServo.setPosition(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STARTPOS_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathStarttoShoot);
                    pathState = PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_SECONDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoottoSecond);
                    pathState = PathState.COLLECT_SECONDROW;
                    waiting = false;
                    intake.setPower(1);

                }
                break;
            case COLLECT_SECONDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathSecondCollect, collectSpeed, true);
                    pathState = PathState.SECONDROW_SHOOTPOS;
                    secondrow = true;
                    waiting = false;

                }
                break;
            case SECONDROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathSecondtoShoot);
                    pathState = PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_THIRDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoottoThird);
                    pathState = PathState.COLLECT_THIRDROW;
                    waiting = false;
                    intake.setPower(1);

                }
                break;
            case COLLECT_THIRDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathThirdCollect, collectSpeed, true);
                    pathState = PathState.THIRDROW_SHOOTPOS;
                    thirdrow = true;
                    waiting = false;

                }
                break;
            case THIRDROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathThirdtoShoot);
                    pathState = PathState.SHOOT;
                    waiting = false;
                }
                break;

            case SHOOTPOS_ENDPOSE:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoottoEnd);
                    pathState = PathState.STOP;
                    waiting = false;
                }
                break;
            case SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    if (llResult != null && llResult.isValid()){
                        Pose3D botPose = llResult.getBotpose();
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        pid_turn_by_gyro(llResult.getTx(), 0.5);
                        telemetry.update();
                    }
                    //sleep(500);
                    shoot();
//                    follower.update();
                    intake.setPower(0);
                    sleep(200);
                    intake.setPower(1);
                    sleep(250);
                    shoot();
//                    follower.update();
                    sleep(250);
                    shoot();
//                    follower.update();
                    if(belly){
                        pathState = PathState.SHOOTPOS_SECONDROW;
                        belly = false;
                    }
                    else if(secondrow) {
                        pathState = PathState.SHOOTPOS_THIRDROW;
                        secondrow = false;
                    }
                    else if(thirdrow) {
                        pathState = PathState.SHOOTPOS_ENDPOSE;
                        thirdrow = false;
                    }
                    waiting = false;
                }
                break;
            case STOP:
                stopall();
                break;
            default:
                break;
        }
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
    }
    private double getHeading() {
        double angle = getRawHeading();
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public void pid_turn_by_gyro(double targetYaw, double speed){
        targetYaw=-targetYaw;
        double currentYaw = getHeading();
        double error;
        double actYaw = getHeading() + targetYaw;
        double kp = 0.01;
        while(Math.abs(actYaw - getHeading()) > 0.85){

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
            telemetry.update();
            setDrivePower(-power, -power, power, power );
        }
        setDrivePower(0,0,0,0 );

    }

    private void setDrivePower(double lfPower, double lrPower, double rfPower, double rrPower)
    {
        leftFront.setPower(lfPower);
        leftRear.setPower(lrPower);
        rightFront.setPower(rfPower);
        rightRear.setPower(rrPower);
    }
}
