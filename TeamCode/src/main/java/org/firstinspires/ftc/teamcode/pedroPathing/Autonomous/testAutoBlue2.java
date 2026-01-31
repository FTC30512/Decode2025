package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static android.os.SystemClock.sleep;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
    private LLResult llResult;
    public Follower follower;

    public AutonomousConstants constants;
    public AutonomousMovement movement;
    public AutonomousImplements implement;
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
    public enum AutoVariations {
        FIRSTROW,
        SECONDROW,
        THIRDROW,
        SECONDROW_OPEN_GATE,
        ENDPOSE
    }
    AutoVariations[] autoVariations = new AutoVariations[5];
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
    private ColorSensor colorSensor;
    private YawPitchRollAngles orientation;
    int idx = 0;

    @Override
    public void init() {
        initHardware();
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(1.0);

        buildPaths();
        constants.pathState = AutonomousConstants.PathState.STARTPOS_SHOOTPOS;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // --- Hardware initialization ---
    private void initHardware() {
        movement.init(hardwareMap);
        implement.init(constants, hardwareMap);
    }

    @Override
    public void init_loop() {
        orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();
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
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
            default:
                autoVariations[0] = AutoVariations.THIRDROW;
                autoVariations[1] = AutoVariations.SECONDROW;
                autoVariations[2] = AutoVariations.ENDPOSE;
                break;
        }
    }

    @Override
    public void start() {
        implement.setLimelightPipeline(1);
        constants.pathState = AutonomousConstants.PathState.STARTPOS_SHOOTPOS;
        implement.setIntakePower(1);
        implement.setShooterVelocity(constants.farShooterSpeed);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();
        telemetry.addData("path state", constants.pathState);
        telemetry.addData("IdNum", IdNum);
        telemetry.addData("IDX", idx);
        telemetry.addData("Length", autoVariations.length);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("is Busy ", follower.isBusy());
        telemetry.addData("is Turning ", follower.isTurning());
        telemetry.addData("shooter velocity ", implement.getShooterVelocity());
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
                    follower.followPath(pathSecondtoShoot, 0.8, false);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
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
                    constants.pathState = AutonomousConstants.PathState.COLLECT_THIRDROW;
                    waiting = false;
                    implement.setIntakePower(1);
                }
                break;
            case COLLECT_THIRDROW:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathThirdCollect, constants.collectSpeed, true);
                    constants.pathState = AutonomousConstants.PathState.THIRDROW_SHOOTPOS;
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
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
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
                    follower.followPath(pathGateFinaltoShoot);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;

            case SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                    follower.breakFollowing();
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    if (llResult != null && llResult.isValid()){
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        movement.pid_turn_by_gyro(llResult.getTx(), 0.5);
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
