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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous(name = "AutoBlue1", group = "Autonomous")
public class testAutoBlue1 extends OpMode {

    //19.6, 126.6, 140.3
    public Follower follower;

    public AutonomousConstants constants = new AutonomousConstants();
    public AutonomousMovement movement = new AutonomousMovement();
    public AutonomousImplements implement = new AutonomousImplements();
    //private final Pose initPose = new Pose(39.5, 138.5, Math.toRadians(90));
    private final Pose startPose = new Pose(19.6, 126.6, Math.toRadians(140.3));
    private final Pose shootPose = new Pose(57.000, 86.000, Math.toRadians(135));
    private final Pose firstRowStartPose = new Pose(52, 87.000, Math.toRadians(0));
    private final Pose firstRowEndPose = new Pose(20, 87.000, Math.toRadians(0));
    private final Pose secondRowStartPose = new Pose(48, 63.500, Math.toRadians(0));
    private final Pose secondRowEndPose = new Pose(13, 63.500, Math.toRadians(0));
    private final Pose gatePoseHalf = new Pose(26, 71.5, Math.toRadians(90));
    private final Pose gatePoseFinal = new Pose(17, 71.5, Math.toRadians(90));
    private final Pose thirdRowStartPose = new Pose(44.000, 38.000, Math.toRadians(0));
    private final Pose thirdRowEndPose = new Pose(22.000, 38.000, Math.toRadians(0));
    //    private final Pose endPose = new Pose(48, 128, Math.toRadians(180));
    private final Pose endPose = new Pose(48, 72, Math.toRadians(180));
    public enum AutoVariations {
        FIRSTROW,
        SECONDROW,
        THIRDROW,
        SECONDROW_OPEN_GATE,
        ENDPOSE
    }
    public PathChain
            pathStarttoShoot,
            pathShoottoSecond,
            pathSecondCollect,
            pathSecondtoShoot,
            pathShoottoFirst,
            pathFirstCollect,
            pathFirsttoShoot,
            pathShoottoEnd,
            pathSecondtoGateHalf,
            pathGateHalftoGateFinal,
            pathGateFinaltoShoot;
    private int IdNum = 21;
    private LLResult llResult;
    private YawPitchRollAngles orientation;
    AutoVariations[] autoVariations = new AutoVariations[5];
    int idx = 0;

    @Override
    public void init() {
        initHardware();
        implement.setLimelightPipeline(2);

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(initPose);
        follower.setMaxPower(1.0);

        buildPaths();
        constants.pathState = AutonomousConstants.PathState.STARTPOS_SHOOTPOS;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        autoVariations[0] = AutoVariations.SECONDROW;
        autoVariations[1] = AutoVariations.FIRSTROW;
        autoVariations[2] = AutoVariations.ENDPOSE;

    }
    // --- Hardware initialization ---
    private void initHardware() {
        movement.init(hardwareMap);
        implement.init(constants, hardwareMap);
    }


    @Override
    public void init_loop() {
        //follower.update();
        orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();
        if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
            IdNum = llResult.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("AprilTag ID", IdNum);
        } else {
            telemetry.addLine("No AprilTag detected");
        }

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getHeading());
        telemetry.update();

        switch (IdNum) {
            case 0:
                autoVariations[0] = AutoVariations.SECONDROW;
                autoVariations[1] = AutoVariations.FIRSTROW;
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
            case 1:
                autoVariations[0] = AutoVariations.SECONDROW_OPEN_GATE;
                autoVariations[1] = AutoVariations.FIRSTROW;
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
            default:
                break;
        }
    }
    @Override
    public void start() {
        follower.setStartingPose(startPose);
        constants.pathState = AutonomousConstants.PathState.STARTPOS_SHOOTPOS;
        implement.setLimelightPipeline(1);
        implement.setIntakePower(1);
        implement.setShooterVelocity(constants.nearShooterSpeed);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();

        telemetry.addData("path state", constants.pathState);
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
                        new BezierLine(
                                secondRowEndPose,
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
                .setHeadingConstraint(0.0001)
                .build();

        pathShoottoEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, endPose)
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
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
                                new Pose(64, 63),
                                shootPose
                        )
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(gatePoseFinal.getHeading(), shootPose.getHeading())
                .build();
    }
    long waitStart = 0;
    boolean waiting = false;

    public void autonomousPathUpdate() {
        switch (constants.pathState) {
            case STARTPOS_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathStarttoShoot);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
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
                    constants.pathState = AutonomousConstants.PathState.COLLECT_SECONDROW;
                    waiting = false;
                    implement.setIntakePower(1);
                }
                break;
            case COLLECT_SECONDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathSecondCollect, constants.collectSpeed, true);
                    boolean contains = Arrays.asList(autoVariations).contains(AutoVariations.SECONDROW_OPEN_GATE);

                    if (contains) {
                        constants.pathState = AutonomousConstants.PathState.SECONDROW_GATEHALF;
                    } else {
                        constants.pathState = AutonomousConstants.PathState.SECONDROW_SHOOTPOS;
                    }
                    waiting = false;

                }
                break;
            case SECONDROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathSecondtoShoot);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_FIRSTROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathShoottoFirst);
                    constants.pathState = AutonomousConstants.PathState.COLLECT_FIRSTROW;
                    waiting = false;
                    implement.setIntakePower(1);
                }
                break;
            case COLLECT_FIRSTROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathFirstCollect, constants.collectSpeed, false);
                    constants.pathState = AutonomousConstants.PathState.FIRSTROW_SHOOTPOS;
                    waiting = false;

                }
                break;
            case FIRSTROW_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathFirsttoShoot, 0.9, false);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_THIRDROW:
                break;
            case COLLECT_THIRDROW:
                break;
            case THIRDROW_SHOOTPOS:
                break;
            case SHOOTPOS_ENDPOSE:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathShoottoEnd);
                    constants.pathState = AutonomousConstants.PathState.STOP;
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
                    constants.pathState = AutonomousConstants.PathState.GATEHALF_GATEFINAL;
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
                    constants.pathState = AutonomousConstants.PathState.GATEFINAL_SHOOT;
                    waiting = false;
                }
                break;

            case GATEFINAL_SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 1000) {
                    follower.followPath(pathGateFinaltoShoot, 0.9, false);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    if (llResult != null && llResult.isValid()){
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        movement.pid_turn_by_gyro(llResult.getTx(), 0.0,0.5);
                        telemetry.update();
                    }
                    implement.shoot();
                    implement.setIntakePower(-0.15);
                    sleep(150);
                    implement.setIntakePower(1);
                    sleep(250);
                    implement.shoot();
                    sleep(250);
                    implement.shoot();
                    if (autoVariations[idx] == AutoVariations.FIRSTROW) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_FIRSTROW;
                    } else if (autoVariations[idx] == AutoVariations.SECONDROW) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_SECONDROW;
                    } else if (autoVariations[idx] == AutoVariations.THIRDROW) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_THIRDROW;
                    } else if (autoVariations[idx] == AutoVariations.SECONDROW_OPEN_GATE) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_SECONDROW;
                    } else if (autoVariations[idx] == AutoVariations.ENDPOSE) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_ENDPOSE;
                    }
                    idx += 1;
                    waiting = false;
                }
                break;
            case STOP:
                implement.stopall();
                break;
            default:
                break;
        }
    }
}