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

@Autonomous(name = "Blue2Auto")
public class Blue2Auto extends OpMode {

    public static PathConstraints pathConstraints = new PathConstraints
            (0.3, 100, 4, 5);

    public Follower follower;
    private int pathState;


    private final Pose startPose = new Pose(49, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.429, 12.319, Math.toRadians(120));

    // Cleaned & Renamed Path Variables
    public PathChain pathShoot1;          // Path1
    public PathChain pathToFirstRow;      // Path2
    public PathChain pathBackFirstRow;    // Path3
    public PathChain pathShoot3;          // Path4
    public PathChain pathToSecondRow;     // Path5
    public PathChain pathBackSecondRow;   // Path6
    public PathChain pathShoot2;          // Path7
    public PathChain pathExit;            // Path8

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
        follower.followPath(pathShoot1); // Start pose to shoot
        pathState = 2;
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
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        pathToFirstRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(43.949, 35.292)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                .build();

        pathBackFirstRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(new Pose(43.949, 35.292), new Pose(12.822, 35.292)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pathShoot3 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(12.822, 35.958),
                        new Pose(20.310, 16.314),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                .build();

        pathToSecondRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(46.451, 58.598)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                .build();

        pathBackSecondRow = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(new Pose(46.451, 58), new Pose(17.313, 58.598)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pathShoot2 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(17.313, 58.598),
                        new Pose(57.433526011560694, 61.928323699421966),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                .build();

        pathExit = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(32.795, 12.985)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();
    }

    long waitStart = 0;
    boolean waiting = false;

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 2: // After Shoot1 → Go to second row
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

            case 3: // Back up in second row
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

            case 4: // Shoot2
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

            case 5: // Go to first row
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

            case 6: // Back up in first row
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

            case 7: // Shoot3
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

            case 8: // Exit
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

            case 9: // Finished
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
