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

@Autonomous(name = "Red2Auto")
public class Red2Auto extends OpMode {
    public static PathConstraints pathConstraints = new PathConstraints
            (0.3, 100, 4, 5);

    public Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(144-49, 8, Math.toRadians(90));

    private final Pose shootPose = new Pose(144-61.429, 12.319, Math.toRadians(60));

    // Path variables
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

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
        follower.followPath(Path1); //Start pose to shoot
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

        Path1 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                .build();

        Path2 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(144-43.949, 35.292)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(new Pose(144-43.949, 35.292), new Pose(144-12.822, 35.292)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(144-12.822, 35.958),
                        new Pose(144-20.310, 16.314),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60))
                .build();

        Path5 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(144-46.451, 55.598)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(new Pose(144-46.451, 55.598), new Pose(144-17.313, 55.598)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(144-17.313, 55.598),
                        new Pose(144-32.296, 19.810),
                        shootPose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60))
                .build();

        Path8 = follower.pathBuilder()
                .setConstraints(pathConstraints)
                .addPath(new BezierLine(shootPose, new Pose(144-32.795, 12.985)))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();
    }

    long waitStart = 0;
    boolean waiting = false;

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 2: // Go to first row
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) { // 1 second pause
                    follower.followPath(Path2);
                    pathState = 3;
                    waiting = false;
                }
                break;

            case 3: //Back up into first row
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path3);
                    pathState = 4;
                    waiting = false;
                }
                break;

            case 4: //Go to shoot
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path4);
                    pathState = 5;
                    waiting = false;
                }
                break;

            case 5: //Go up to second row
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path5);
                    pathState = 6;
                    waiting = false;
                }
                break;

            case 6: //Back up into second row
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path6);
                    pathState = 7;
                    waiting = false;
                }
                break;

            case 7: //Go to shoot
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path7);
                    pathState = 8;
                    waiting = false;
                }
                break;

            case 8: //Leave base
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(Path8);
                    pathState = 9;
                    waiting = false;
                }
                break;

            case 9: //Done
                if (!follower.isBusy() && !waiting) {
                    waitStart = System.currentTimeMillis();
                    waiting = true;
                }
                if (waiting && System.currentTimeMillis() - waitStart >= 500) {
                    pathState = 10; // DONE
                    waiting = false;
                }
                break;
        }
    }
}
