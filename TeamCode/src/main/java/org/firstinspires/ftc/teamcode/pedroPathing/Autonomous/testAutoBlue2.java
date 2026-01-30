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
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "AutoBlue2", group = "Autonomous")
public class testAutoBlue2 extends OpMode {

    //public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);
    private LLResult llResult;

    public Follower follower;
    private final Pose startPose = new Pose(56.500, 11.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.000, 21.000, Math.toRadians(110));
    private final Pose firstRowStartPose = new Pose(44.000, 87.000, Math.toRadians(0));
    private final Pose firstRowEndPose = new Pose(25.000, 87.000, Math.toRadians(0));
    private final Pose secondRowStartPose = new Pose(46.000, 60.000, Math.toRadians(0));
    private final Pose secondRowEndPose = new Pose(15, 60.000, Math.toRadians(0));
    private final Pose thirdRowStartPose = new Pose(46.000, 38.000, Math.toRadians(0));
    private final Pose thirdRowEndPose = new Pose(15, 38.000, Math.toRadians(0));
    private final Pose gatePoseHalf = new Pose(24, 67, Math.toRadians(90));
    private final Pose gatePoseFinal = new Pose(16, 67, Math.toRadians(90));
    private final Pose endPose = new Pose(38.71408250355619, 33.5931721194879, Math.toRadians(180));
    double collectSpeed = 0.65;
    private enum PathState{
        STARTPOS_SHOOTPOS,
        SHOOTPOS_FIRSTROW,
        SHOOTPOS_SECONDROW,
        SHOOTPOS_THIRDROW,
        COLLECT_FIRSTROW,
        COLLECT_SECONDROW,
        SECONDROW_GATEHALF,
        GATEHALF_GATEFINAL,
        GATEFINAL_SHOOT,
        COLLECT_THIRDROW,
        FIRSTROW_SHOOTPOS,
        SECONDROW_SHOOTPOS,
        THIRDROW_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT,
        STOP
    }
    public enum AutoVariations {
        FIRSTROW,
        SECONDROW,
        THIRDROW,
        SECONDROW_OPEN_GATE,
        ENDPOSE
    }

    PathState pathState;
    private int IdNum = 21;
    public PathChain
            pathStarttoShoot,
            pathShoottoSecond,
            pathSecondCollect,
            pathSecondtoGateHalf,
            pathGateHalftoGateFinal,
            pathGateFinaltoShoot,
            pathSecondtoShoot,
            pathShoottoThird,
            pathThirdCollect,
            pathThirdtoShoot,
            pathShoottoEnd;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private ColorSensor colorSensor;
    private YawPitchRollAngles orientation;

    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Limelight3A limelight;
    private int shooterSpeed = 2525;
    private double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    //private double Kp = 25.0, Ki = 3.0, Kd = 0.0, Kf = 2.8;
    private IMU imu;
    boolean belly, firstrow, secondrow, thirdrow = false;
    AutoVariations[] autoVariations = new AutoVariations[5];
    int idx = 0;

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
        limelight.pipelineSwitch(2);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight.start();
    }

    @Override
    public void init_loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
            IdNum = llResult.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("AprilTag ID", IdNum);
        } else {
            telemetry.addLine("No AprilTag detected");
        }
        telemetry.update();

        switch (IdNum){
            case 21:
                autoVariations[0] = AutoVariations.THIRDROW;
                autoVariations[1] = AutoVariations.SECONDROW;
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
            case 22:
                autoVariations[0] = AutoVariations.THIRDROW;
                autoVariations[1] = AutoVariations.SECONDROW_OPEN_GATE;
                autoVariations[3] = AutoVariations.ENDPOSE;
                break;
            default:
                autoVariations[0] = AutoVariations.THIRDROW;
                autoVariations[1] = AutoVariations.SECONDROW;
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
        }
        
//        if (IdNum == 21){
//            autoVariations[0] = AutoVariations.THIRDROW;
//            autoVariations[1] = AutoVariations.SECONDROW;
//            autoVariations[2] = AutoVariations.ENDPOSE;
//        }
//
//        if (IdNum == 22){
//            autoVariations[0] = AutoVariations.SECONDROW_OPEN_GATE;
//            autoVariations[1] = AutoVariations.THIRDROW;
//            autoVariations[3] = AutoVariations.ENDPOSE;
//        }
    }

    @Override
    public void start() {
        limelight.pipelineSwitch(1);
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
        telemetry.addData("IdNum", IdNum);
        telemetry.addData("IDX", idx);
        telemetry.addData("Length", autoVariations.length);
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

        
        pathShoottoSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, secondRowStartPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowStartPose.getHeading())
                .build();
        
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
                                new Pose(52.665, 53.268),
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
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(thirdRowEndPose.getHeading(), shootPose.getHeading())
                .build();

        pathSecondtoGateHalf = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(secondRowEndPose, gatePoseHalf)
                )
                .setLinearHeadingInterpolation(secondRowEndPose.getHeading(), gatePoseHalf.getHeading())
                .build();

        pathGateHalftoGateFinal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(gatePoseHalf, gatePoseFinal)
                )
                .setLinearHeadingInterpolation(gatePoseHalf.getHeading(), gatePoseFinal.getHeading())
                .build();

        pathGateFinaltoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                gatePoseFinal,
                                new Pose(52.665, 53.268),
                                shootPose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(gatePoseFinal.getHeading(), shootPose.getHeading())
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathSecondCollect, collectSpeed, true);

                    boolean contains = Arrays.asList(autoVariations).contains(AutoVariations.SECONDROW_OPEN_GATE);

                    if (contains) {
                        pathState = PathState.SECONDROW_GATEHALF;
                    } else {
                        pathState = PathState.SECONDROW_SHOOTPOS;
                    }
                    secondrow = true;
                    waiting = false;

                }
                break;
            case SECONDROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathSecondtoShoot, 0.8, false);
                    pathState = PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_THIRDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathShoottoEnd);
                    pathState = PathState.STOP;
                    waiting = false;
                }
                break;

            case SECONDROW_GATEHALF:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathSecondtoGateHalf);
                    pathState = PathState.GATEHALF_GATEFINAL;
                    waiting = false;
                }
                break;

            case GATEHALF_GATEFINAL:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathGateHalftoGateFinal);
                    pathState = PathState.GATEFINAL_SHOOT;
                    waiting = false;
                }
                break;

            case GATEFINAL_SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 1000) {
                    follower.followPath(pathGateFinaltoShoot);
                    pathState = PathState.SHOOT;
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
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
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
                    intake.setPower(-0.15);
                    sleep(150);
                    intake.setPower(1);
                    sleep(250);
                    shoot();
//                    follower.update();
                    sleep(250);
                    shoot();
//                    follower.update();

                    if (autoVariations[idx] == AutoVariations.FIRSTROW) {
                        pathState = PathState.SHOOTPOS_FIRSTROW;
                    } else if (autoVariations[idx] == AutoVariations.SECONDROW) {
                        pathState = PathState.SHOOTPOS_SECONDROW;
                    } else if (autoVariations[idx] == AutoVariations.THIRDROW) {
                        pathState = PathState.SHOOTPOS_THIRDROW;
                    } else if (autoVariations[idx] == AutoVariations.SECONDROW_OPEN_GATE) {
                        pathState = PathState.SHOOTPOS_SECONDROW;
                    } else if (autoVariations[idx] == AutoVariations.ENDPOSE) {
                        pathState = PathState.SHOOTPOS_ENDPOSE;
                    }
                    idx += 1;


//                    if(belly){
//                        pathState = PathState.SHOOTPOS_SECONDROW;
//                        belly = false;
//                    }
//                    else if(secondrow) {
//                        pathState = PathState.SHOOTPOS_THIRDROW;
//                        secondrow = false;
//                    }
//                    else if(thirdrow) {
//                        pathState = PathState.SHOOTPOS_ENDPOSE;
//                        thirdrow = false;
//                    }
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
