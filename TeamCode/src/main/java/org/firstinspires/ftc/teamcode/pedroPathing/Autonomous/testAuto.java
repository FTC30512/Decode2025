package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Autonomous(name = "Test Auto", group = "Autonomous")
public class testAuto extends OpMode {

    public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private final Pose startPose = new Pose(56.500, 11.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.000, 19.000, Math.toRadians(125));
    private final Pose firstRowStartPose = new Pose(44.000, 84.000, Math.toRadians(0));
    private final Pose firstRowEndPose = new Pose(22.000, 84.000, Math.toRadians(0));
    private final Pose secondRowStartPose = new Pose(44.000, 60.000, Math.toRadians(0));
    private final Pose secondRowEndPose = new Pose(22.000, 60.000, Math.toRadians(0));
    private final Pose thirdRowStartPose = new Pose(44.000, 35.700, Math.toRadians(0));
    private final Pose thirdRowEndPose = new Pose(22.000, 35.700, Math.toRadians(0));
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
        SECONDROW_SHOOTPOS,
        THIRDROW_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT

    }
    PathState pathState;
    public PathChain pathStarttoShoot, pathShoottoSecond, pathSecondCollect, pathSecondtoShoot, pathShoottoEnd;
    private Servo gateServo, shooterServo;
    private DcMotor intake, shooter;
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
        shooter = hardwareMap.dcMotor.get("Shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

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
    }

    @Override
    public void loop() {
        intake.setPower(1);
        shooter.setPower(0.9);
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Max Power", follower.getMaxPowerScaling());
        telemetry.update();
    }

    public void buildPaths() {

        pathStarttoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, shootPose)
                )
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
                .setLinearHeadingInterpolation(secondRowEndPose.getHeading(), shootPose.getHeading())
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
            case SHOOTPOS_ENDPOSE:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoottoEnd);
                    waiting = false;
                }
                break;
            case SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                    follower.breakFollowing();
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    sleep(500);
                    shoot();
                    intake.setPower(0);
                    sleep(200);
                    intake.setPower(1);
                    sleep(250);
                    shoot();
                    sleep(250);
                    shoot();
                    if(belly){
                        pathState = PathState.SHOOTPOS_SECONDROW;
                        belly = false;
                    }
                    else if(secondrow) {
                        pathState = PathState.SHOOTPOS_ENDPOSE;
                        secondrow = false;
                    }
                    waiting = false;
                }
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
