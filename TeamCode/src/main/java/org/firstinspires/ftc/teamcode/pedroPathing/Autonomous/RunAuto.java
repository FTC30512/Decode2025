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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;

@Autonomous(name = "Run Auto", group = "Autonomous")
public class RunAuto extends OpMode {

    public Follower follower;
    public AutonomousConstants constants = new AutonomousConstants();
    public AutonomousMovement movement = new AutonomousMovement();
    public AutonomousImplements implement = new AutonomousImplements();
    AutoCoordinateMap autoMap = constants.AutoBlue2;
    public enum AutoPose {
        AUTO_BLUE_1,
        AUTO_BLUE_2,
        AUTO_RED_1,
        AUTO_RED_2
    }
    public enum AutoVariations {
        FIRSTROW,
        SECONDROW,
        THIRDROW,
        HOME,
        SECONDROW_OPEN_GATE,
        ENDPOSE
    }
    AutoVariations[] autoVariations = new AutoVariations[5];
    int idx = 0;
    public PathChain
            pathStarttoShoot,
            pathShoottoFirst,
            pathFirstCollect,
            pathFirsttoShoot,
            pathShoottoSecond,
            pathSecondCollect,
            pathSecondtoShoot,
            pathShoottoThird,
            pathThirdCollect,
            pathThirdtoShoot,
            pathShoottoHome,
            pathHomeCollect,
            pathHometoShoot,
            pathShoottoEnd,
            pathSecondtoGateHalf,
            pathGateHalftoGateFinal,
            pathGateFinaltoShoot;
    private LLResult llResult;
    private AutoPose autoPose = AutoPose.AUTO_BLUE_2;
    private int IdNum = 25;
    double first_to_shoot_speed = 1.0,
            second_to_shoot_speed = 1.0,
            third_to_shoot_speed = 1.0,
            gate_to_shoot_speed = 1.0,
            collectSpeed = 0.65;
    double offset = 0.0;
    boolean break_following_after_shoot = true;
    @Override
    public void init(){
        implement.init(constants, hardwareMap);
        movement.init(hardwareMap);
        implement.setLimelightPipeline(2);
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        autoVariations[0] = AutoVariations.SECONDROW;
        autoVariations[1] = AutoVariations.FIRSTROW;
        autoVariations[2] = AutoVariations.ENDPOSE;
    }
    @Override
    public void init_loop(){
        YawPitchRollAngles orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();
        if (IdNum > 11) {
            if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
                IdNum = llResult.getFiducialResults().get(0).getFiducialId();
                telemetry.addData("AprilTag ID", IdNum);
            } else {
                telemetry.addLine("No AprilTag detected");
            }
        }
    }
    @Override
    public void start() {
        if (IdNum == 0 || IdNum == 1) {
            autoPose = AutoPose.AUTO_BLUE_1;
            autoMap = constants.AutoBlue1;
            Pose gatetoshootcp = new Pose(64, 63);
            buildPaths(null, gatetoshootcp, null);
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
            implement.setShooterVelocity(constants.nearShooterSpeed);
            first_to_shoot_speed = 0.9;
            gate_to_shoot_speed = 0.9;
            offset = 2.0;
        }
        else if (IdNum == 4 || IdNum == 5) {
            autoPose = AutoPose.AUTO_RED_1;
            autoMap = constants.AutoRed1;
//            Pose gatetoshootcp = new Pose(80, 63);
            buildPaths(null, null, null);
            switch (IdNum){
                case 4:
                    autoVariations[0] = AutoVariations.SECONDROW;
                    autoVariations[1] = AutoVariations.FIRSTROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                case 5:
                    autoVariations[0] = AutoVariations.SECONDROW_OPEN_GATE;
                    autoVariations[1] = AutoVariations.FIRSTROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                default:
                    break;
            }
            implement.setShooterVelocity(constants.nearShooterSpeed);
        }
        else if (IdNum == 6 || IdNum == 7 || IdNum == 9) {
            autoPose = AutoPose.AUTO_RED_2;
            autoMap = constants.AutoRed2;
            Pose gatetoshootcp = new Pose(84.40694769008739, 63.48483482567039);
            Pose secondtoshootcp = new Pose(84.40694769008739, 63.48483482567039);
            Pose homebasecp = new Pose(133.76, 33);
            buildPaths(secondtoshootcp, gatetoshootcp, homebasecp);
            switch (IdNum){
                case 6:
                    autoVariations[0] = AutoVariations.SECONDROW;
                    autoVariations[1] = AutoVariations.THIRDROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                case 7:
                    autoVariations[0] = AutoVariations.SECONDROW_OPEN_GATE;
                    autoVariations[1] = AutoVariations.THIRDROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                case 9:
                    autoVariations[0] = AutoVariations.THIRDROW;
                    autoVariations[1] = AutoVariations.HOME;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                default:
                    break;
            }

            implement.setShooterVelocity(constants.farShooterSpeed);
            offset = 2.0;
        }
        else{
            autoPose = AutoPose.AUTO_BLUE_2;
            autoMap = constants.AutoBlue2;
            Pose gatetoshootcp = new Pose(52.665, 53.268);
            Pose secondtoshootcp = new Pose(52.665, 53.268);
            Pose homebasecp = new Pose(10.24, 33);
            buildPaths(secondtoshootcp, gatetoshootcp, homebasecp);
            switch (IdNum){
                case 2:
                    autoVariations[0] = AutoVariations.SECONDROW;
                    autoVariations[1] = AutoVariations.THIRDROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                case 3:
                    autoVariations[0] = AutoVariations.SECONDROW_OPEN_GATE;
                    autoVariations[1] = AutoVariations.THIRDROW;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                    break;
                case 8:
                    autoVariations[0] = AutoVariations.THIRDROW;
                    autoVariations[1] = AutoVariations.HOME;
                    autoVariations[2] = AutoVariations.ENDPOSE;
                default:
                    break;
            }

            implement.setShooterVelocity(constants.farShooterSpeed);
//            third_to_shoot_speed = 0.8;
//            second_to_shoot_speed = 0.8;
        }

        constants.pathState = AutonomousConstants.PathState.STARTPOS_SHOOTPOS;
        follower.setStartingPose(autoMap.startPose);
        implement.setLimelightPipeline(1);
        implement.setIntakePower(1);
    }
    @Override
    public void loop(){
        YawPitchRollAngles orientation = movement.imu.getRobotYawPitchRollAngles();
        implement.updatelimelightOrientation(orientation.getYaw(AngleUnit.DEGREES));
        llResult = implement.getLimelightResult();

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", constants.pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Max Power", follower.getMaxPowerScaling());
        telemetry.update();
    }
    public void buildPaths(Pose second_to_shoot_cp, Pose gate_to_shoot_cp, Pose homebasecp) {

        pathStarttoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.startPose, autoMap.shootPose)
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(autoMap.startPose.getHeading(), autoMap.shootPose.getHeading())
                .build();


        pathShoottoSecond = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.shootPose, autoMap.secondRowStartPose)
                )
                .setLinearHeadingInterpolation(autoMap.shootPose.getHeading(), autoMap.secondRowStartPose.getHeading())
                .build();

        pathSecondCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.secondRowStartPose, autoMap.secondRowEndPose)
                )
                .setLinearHeadingInterpolation(autoMap.secondRowStartPose.getHeading(), autoMap.secondRowEndPose.getHeading())
                .build();
        if (second_to_shoot_cp != null)
        {
            pathSecondtoShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    autoMap.secondRowEndPose,
                                    second_to_shoot_cp,
                                    autoMap.shootPose
                            )
                    )
                    .setHeadingConstraint(0.0001)
                    .setLinearHeadingInterpolation(autoMap.secondRowEndPose.getHeading(), autoMap.shootPose.getHeading())
                    .build();
        }
        else
        {
            pathSecondtoShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    autoMap.secondRowEndPose,
                                    autoMap.shootPose
                            )
                    )
                    .setHeadingConstraint(0.0001)
                    .setLinearHeadingInterpolation(autoMap.secondRowEndPose.getHeading(), autoMap.shootPose.getHeading())
                    .build();
        }

        pathShoottoFirst = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.shootPose, autoMap.firstRowStartPose)
                )
                .setLinearHeadingInterpolation(autoMap.shootPose.getHeading(), autoMap.firstRowStartPose.getHeading())
                .build();

        pathFirstCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.firstRowStartPose, autoMap.firstRowEndPose)
                )
                .setLinearHeadingInterpolation(autoMap.firstRowStartPose.getHeading(), autoMap.firstRowEndPose.getHeading())
                .build();

        pathFirsttoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.firstRowEndPose, autoMap.shootPose)
                )
                .setLinearHeadingInterpolation(autoMap.firstRowEndPose.getHeading(), autoMap.shootPose.getHeading())
                .setHeadingConstraint(0.0001)
                .build();

        pathShoottoThird = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.shootPose, autoMap.thirdRowStartPose)
                )
                .setLinearHeadingInterpolation(autoMap.shootPose.getHeading(), autoMap.thirdRowStartPose.getHeading())
                .build();

        pathThirdCollect = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.thirdRowStartPose, autoMap.thirdRowEndPose)
                )
                .setLinearHeadingInterpolation(autoMap.thirdRowStartPose.getHeading(), autoMap.thirdRowEndPose.getHeading())
                .build();

        pathThirdtoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.thirdRowEndPose, autoMap.shootPose)
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(autoMap.thirdRowEndPose.getHeading(), autoMap.shootPose.getHeading())
                .build();

        pathShoottoHome = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.shootPose, autoMap.homeBaseStartPose)
                )
                .setLinearHeadingInterpolation(autoMap.shootPose.getHeading(), autoMap.homeBaseStartPose.getHeading())
                .build();
        if (homebasecp != null) {
            pathHomeCollect = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(autoMap.homeBaseStartPose, homebasecp, autoMap.homeBaseEndPose)
                    )
                    .setLinearHeadingInterpolation(autoMap.homeBaseStartPose.getHeading(), autoMap.homeBaseEndPose.getHeading())
                    .build();
        }

        pathHometoShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.homeBaseEndPose, autoMap.shootPose)
                )
                .setHeadingConstraint(0.0001)
                .setLinearHeadingInterpolation(autoMap.homeBaseEndPose.getHeading(), autoMap.shootPose.getHeading())
                .build();

        pathShoottoEnd = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.shootPose, autoMap.endPose)
                )
                .setLinearHeadingInterpolation(autoMap.shootPose.getHeading(), autoMap.endPose.getHeading())
                .build();
        pathSecondtoGateHalf = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.secondRowEndPose, autoMap.gatePoseHalf)
                )
                .setLinearHeadingInterpolation(autoMap.secondRowEndPose.getHeading(), autoMap.gatePoseHalf.getHeading())
                .build();

        pathGateHalftoGateFinal = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(autoMap.gatePoseHalf, autoMap.gatePoseFinal)
                )
                .setLinearHeadingInterpolation(autoMap.gatePoseHalf.getHeading(), autoMap.gatePoseFinal.getHeading())
                .build();

        if (gate_to_shoot_cp != null)
        {
            pathGateFinaltoShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    autoMap.gatePoseFinal,
                                    gate_to_shoot_cp,
                                    autoMap.shootPose
                            )
                    )
                    .setHeadingConstraint(0.0001)
                    .setLinearHeadingInterpolation(autoMap.gatePoseFinal.getHeading(), autoMap.shootPose.getHeading())
                    .build();
        }
        else
        {
            pathGateFinaltoShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    autoMap.gatePoseFinal,
                                    autoMap.shootPose
                            )
                    )
                    .setHeadingConstraint(0.0001)
                    .setLinearHeadingInterpolation(autoMap.gatePoseFinal.getHeading(), autoMap.shootPose.getHeading())
                    .build();
        }
    }

    long waitStart = 0;
    boolean waiting = false;
    public void autonomousPathUpdate() {
        boolean hold = false;
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
                    follower.followPath(pathSecondCollect, collectSpeed, true);
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
                    if (second_to_shoot_speed < 1.0) hold = true;
                    follower.followPath(pathSecondtoShoot, second_to_shoot_speed, hold);
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
                    follower.followPath(pathFirstCollect, collectSpeed, false);
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
                    if (first_to_shoot_speed < 1.0) hold = true;
                    follower.followPath(pathFirsttoShoot, first_to_shoot_speed, hold);
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
                    follower.followPath(pathThirdCollect, collectSpeed, true);
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
                    if (third_to_shoot_speed < 1.0) hold = true;
                    follower.followPath(pathThirdtoShoot, third_to_shoot_speed, hold);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOTPOS_HOME:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathShoottoHome);
                    constants.pathState = AutonomousConstants.PathState.COLLECT_HOME;
                    waiting = false;
                    implement.setIntakePower(1);
                }
                break;
            case COLLECT_HOME:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathHomeCollect, 0.5, true);
                    constants.pathState = AutonomousConstants.PathState.HOME_SHOOTPOS;
                    waiting = false;

                }
                break;
            case HOME_SHOOTPOS:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(pathHometoShoot);
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
                    if (gate_to_shoot_speed < 1.0) hold = true;
                    follower.followPath(pathGateFinaltoShoot, gate_to_shoot_speed, hold);
                    constants.pathState = AutonomousConstants.PathState.SHOOT;
                    waiting = false;
                }
                break;
            case SHOOT:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                    if(break_following_after_shoot)
                    {
                        follower.breakFollowing();
                    }
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 100) {
                    if (llResult != null && llResult.isValid()){
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        movement.pid_turn_by_gyro(llResult.getTx(), offset, 0.5);
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
                    } else if (autoVariations[idx] == AutoVariations.HOME) {
                        constants.pathState = AutonomousConstants.PathState.SHOOTPOS_HOME;
                    }else if (autoVariations[idx] == AutoVariations.ENDPOSE) {
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
