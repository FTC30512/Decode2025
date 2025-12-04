package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Blue1Auto extends OpMode {

    public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(20.809, 123.524, Math.toRadians(-35));
    private final Pose shootPose = new Pose(58.932, 84.236, Math.toRadians(135));

    public PathChain pathShoot1;
    public PathChain pathToFirstRow;
    public PathChain pathBackFirstRow;
    public PathChain pathShoot3;
    public PathChain pathToSecondRow;
    public PathChain pathBackSecondRow;
    public PathChain pathShoot2;
    public PathChain pathExit;

    @Override
    public void init() {

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        pathState = 1;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void buildPaths() {

        pathShoot1 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(20.809, 123.524),
                        new Pose(58.932, 84.236)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(135))
                .build();

        pathToSecondRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(58.932, 84.236),
                        new Pose(42.284, 59.931)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        pathBackSecondRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(42.284, 59.931),
                        new Pose(16.985, 59.598)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pathShoot2 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(16.985, 59.598),
                        new Pose(63.593, 55.602),
                        new Pose(58.932, 84.402)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        pathToFirstRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(58.932, 84.402),
                        new Pose(41.951, 84.236)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        pathBackFirstRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(41.951, 84.236),
                        new Pose(16.816, 83.903)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pathShoot3 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(16.816, 83.903),
                        new Pose(58.932, 84.402)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        pathExit = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(
                        new Pose(58.932, 84.402),
                        new Pose(47.778, 67.588)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }

    long waitStart = 0;
    boolean waiting = false;

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 1:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoot1);
                    pathState = 2;
                    waiting = false;
                }
                break;

            case 2:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathToSecondRow);
                    pathState = 3;
                    waiting = false;
                }
                break;

            case 3:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathBackSecondRow);
                    pathState = 4;
                    waiting = false;
                }
                break;

            case 4:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoot2);
                    pathState = 5;
                    waiting = false;
                }
                break;

            case 5:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathToFirstRow);
                    pathState = 6;
                    waiting = false;
                }
                break;

            case 6:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathBackFirstRow);
                    pathState = 7;
                    waiting = false;
                }
                break;

            case 7:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathShoot3);
                    pathState = 8;
                    waiting = false;
                }
                break;

            case 8:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(pathExit);
                    pathState = 9;
                    waiting = false;
                }
                break;

            case 9:
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    pathState = 10;
                    waiting = false;
                }
                break;
        }
    }
}
