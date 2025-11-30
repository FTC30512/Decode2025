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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue2Auto")
public class Blue2Auto extends LinearOpMode {

    public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private int pathState;

    Servo shooterServo, gateServo;
    DcMotor intake, shooter;

    private final Pose startPose = new Pose(49, 8, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.429, 12.319, Math.toRadians(120));

    public PathChain pathShoot1;
    public PathChain pathToFirstRow;
    public PathChain pathBackFirstRow;
    public PathChain pathShoot3;
    public PathChain pathToSecondRow;
    public PathChain pathBackSecondRow;
    public PathChain pathShoot2;
    public PathChain pathExit;

    long waitStart = 0;
    boolean waiting = false;

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        pathState = 1;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();
        if (isStopRequested()) return;

        intake.setPower(0.7);
        shooter.setPower(1);

        waitStart = 0;
        waiting = false;

        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
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

    public void autonomousPathUpdate(){

        switch (pathState) {

            case 1:
                follower.followPath(pathShoot1);
                sleep(500);
                shoot();
                pathState = 2;
                break;

            case 2:
                follower.followPath(pathToSecondRow);
                sleep(500);
                pathState = 3;
                break;

            case 3:
                follower.followPath(pathBackSecondRow);
                sleep(500);
                pathState = 4;
                break;

            case 4:
                follower.followPath(pathShoot2);
                sleep(500);
                shoot();
                pathState = 5;
                break;

            case 5:
                follower.followPath(pathToFirstRow);
                sleep(500);
                pathState = 6;
                break;

            case 6:
                follower.followPath(pathBackFirstRow);
                sleep(500);
                pathState = 7;
                break;

            case 7:
                follower.followPath(pathShoot3);
                sleep(500);
                shoot();
                pathState = 8;
                break;

            case 8:
                follower.followPath(pathExit);
                sleep(500);
                pathState = 9;
                break;

            case 9:
                pathState = 10;
                shooter.setPower(0);
                intake.setPower(0);
                break;
        }
    }

    // --- Shooting method ---
    public void shoot() {
        gateServo.setPosition(0.3);
        sleep(100);
        shooterServo.setPosition(0.35);
        sleep(250);
        shooterServo.setPosition(0);
        sleep(175);
        gateServo.setPosition(0);
        sleep(10);
    }

}
