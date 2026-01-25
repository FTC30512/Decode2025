package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static android.os.SystemClock.sleep;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class AutoBlue2Gate extends OpMode {

    //public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);
    private LLResult llResult;

    public Follower follower;
    private final Pose startPose = new Pose(56.500, 11.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.000, 19.000, Math.toRadians(110));
    private final Pose firstRowStartPose = new Pose(46.000, 84.000, Math.toRadians(0));
    private final Pose firstRowEndPose = new Pose(22.000, 84.000, Math.toRadians(0));
    private final Pose secondRowStartPose = new Pose(46.000, 60.000, Math.toRadians(0));
    private final Pose secondRowEndPose = new Pose(22.000, 60.000, Math.toRadians(0));
    private final Pose secondRowGate1Pose = new Pose(22, 80, Math.toRadians(0));
    private final Pose secondRowGate2Pose = new Pose(18, 80, Math.toRadians(0));
    private final Pose thirdRowStartPose = new Pose(46.000, 38.000, Math.toRadians(0));
    private final Pose thirdRowEndPose = new Pose(22.000, 38.000, Math.toRadians(0));
    private final Pose endPose = new Pose(38.71408250355619, 33.5931721194879, Math.toRadians(180));
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
        GATE1_GATE2,
        GATE2_SHOOTPOS,
        SECONDROW_GATE1,
        THIRDROW_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT,
        STOP

    }
    PathState pathState;
    public PathChain pathStarttoShoot, pathShoottoSecond, pathSecondCollect, pathSecondGate1, pathGate1toGate2 ,pathGate2toShoot, pathShoottoThird, pathThirdCollect, pathThirdtoShoot, pathShoottoEnd;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private YawPitchRollAngles orientation;

    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Limelight3A limelight;
    private int shooterSpeed = 2475;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
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
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf);

        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

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
        follower.update();
        autonomousPathUpdate();

        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = limelight.getLatestResult();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("is Busy ", follower.isBusy());
        telemetry.addData("is Turning ", follower.isTurning());
        telemetry.addData("shooter velocity ", shooter.getVelocity());
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

        pathSecondGate1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                secondRowEndPose,
                                new Pose(22.400, 72.037),
                                secondRowGate1Pose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        pathGate1toGate2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                secondRowGate1Pose,
                                secondRowGate2Pose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(secondRowGate1Pose.getHeading(), secondRowGate2Pose.getHeading())
                .build();

        pathGate2toShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                secondRowGate2Pose,
                                shootPose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(secondRowGate2Pose.getHeading(), shootPose.getHeading())
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
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(thirdRowEndPose.getHeading(), shootPose.getHeading())
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
                    pathState = PathState.SECONDROW_GATE1;
                    secondrow = true;
                    waiting = false;

                }
                break;
            case SECONDROW_GATE1:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathSecondGate1, 0.8, false);
                    pathState = PathState.GATE1_GATE2;
                    waiting = false;
                }
                break;

            case GATE1_GATE2:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathGate1toGate2, 0.8, false);
                    pathState = PathState.GATE2_SHOOTPOS;
                    waiting = false;
                }
                break;
            case GATE2_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathGate2toShoot, 0.8, false);
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
                    follower.followPath(pathThirdtoShoot, 0.8, false);
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
                    /*
                    if (follower.getPose().getHeading() > shootPose.getHeading() + Math.toRadians(2) ||
                            follower.getPose().getHeading() < shootPose.getHeading() - Math.toRadians(2) ){
                        double amount_of_turn = follower.getPose().getHeading() - shootPose.getHeading();
                        boolean isLeft = false;
                        if(amount_of_turn < 0)
                        {
                            isLeft = true;
                            amount_of_turn = -1*amount_of_turn;
                        }

                        follower.turnDegrees(amount_of_turn, isLeft);
                    }
                    else{}
                        */
//                    follower.breakFollowing();
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
