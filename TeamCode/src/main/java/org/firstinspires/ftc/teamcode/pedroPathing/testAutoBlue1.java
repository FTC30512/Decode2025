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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "testAutoBlue1", group = "Autonomous")
public class testAutoBlue1 extends OpMode {

    //public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private final Pose startPose = new Pose(32.5, 136.5, Math.toRadians(90));
    private final Pose shootPose = new Pose(50.000, 93.000, Math.toRadians(135));
    private final Pose firstRowStartPose = new Pose(44.000, 84.000, Math.toRadians(0));
    private final Pose firstRowEndPose = new Pose(25.000, 84.000, Math.toRadians(0));
    private final Pose secondRowStartPose = new Pose(44.000, 60.000, Math.toRadians(0));
    private final Pose secondRowEndPose = new Pose(22.000, 60.000, Math.toRadians(0));
    private final Pose thirdRowStartPose = new Pose(44.000, 38.000, Math.toRadians(0));
    private final Pose thirdRowEndPose = new Pose(22.000, 38.000, Math.toRadians(0));
    private final Pose endPose = new Pose(48, 128, Math.toRadians(180));
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
    public PathChain pathStarttoShoot, pathShoottoSecond, pathSecondCollect, pathSecondtoShoot, pathShoottoFirst, pathFirstCollect, pathFirsttoShoot, pathShoottoEnd;
    private Servo gateServo, shooterServo;
    private DcMotor intake;
    private DcMotorEx shooter;
    private int shooterSpeed = 2100;

    private double Kp = 25.0, Ki = 3.0, Kd = 0.0, Kf = 2.8;
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

        //colorSensor = hardwareMap.colorSensor.get("colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);
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
                                new Pose(47.665, 48.268),
                                shootPose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(secondRowEndPose.getHeading(), shootPose.getHeading())
                .build();

        pathShoottoFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, firstRowStartPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowStartPose.getHeading())
                .build();

        pathFirstCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(firstRowStartPose, firstRowEndPose)
                )
                .setLinearHeadingInterpolation(firstRowStartPose.getHeading(), firstRowEndPose.getHeading())
                .build();

        pathFirsttoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(firstRowEndPose, shootPose)
                )
                .setLinearHeadingInterpolation(firstRowEndPose.getHeading(), shootPose.getHeading())
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
            case SHOOTPOS_FIRSTROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoottoFirst);
                    pathState = PathState.COLLECT_THIRDROW;
                    waiting = false;
                    intake.setPower(1);

                }
                break;
            case COLLECT_FIRSTROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathFirstCollect, collectSpeed, false);
                    pathState = PathState.FIRSTROW_SHOOTPOS;
                    firstrow = true;
                    waiting = false;

                }
                break;
            case FIRSTROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathFirsttoShoot);

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
                    //sleep(500);
                    shoot();
                    follower.update();
                    intake.setPower(0);
                    sleep(200);
                    intake.setPower(1);
                    sleep(250);
                    shoot();
                    follower.update();
                    sleep(250);
                    shoot();
                    follower.update();
                    if(belly){
                        pathState = PathState.SHOOTPOS_SECONDROW;
                        belly = false;
                    }
                    else if(secondrow) {
                        pathState = PathState.SHOOTPOS_FIRSTROW;
                        secondrow = false;
                    }
                    else if(firstrow) {
                        pathState = PathState.SHOOTPOS_ENDPOSE;
                        firstrow = false;

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
}