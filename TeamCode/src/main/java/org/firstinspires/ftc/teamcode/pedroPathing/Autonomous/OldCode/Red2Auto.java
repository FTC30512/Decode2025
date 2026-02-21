package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.OldCode;

import static android.os.SystemClock.sleep;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Red2Auto extends OpMode {

    public static PathConstraints pathConstraints = new PathConstraints(0.3, 100, 4, 5);

    public Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(144 - 48, 9.85, Math.PI - Math.toRadians(90));

    private DcMotor intake, shooter;
    private Servo gateServo, shooterServo;

    public PathChain pathShoot1, pathToFirstRow, pathBackFirstRow,
            pathShoot3, pathToSecondRow, pathBackSecondRow,
            pathShoot2, pathExit;

    @Override
    public void init() {
        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        pathState = 1;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intake = hardwareMap.dcMotor.get("Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);

        gateServo.setPosition(0);
        shooterServo.setPosition(0);
    }

    @Override
    public void start() {
        shooter.setPower(1);
        intake.setPower(1);
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();
        runAutonomousStateMachine();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void buildPaths() {
        // Mirror across x = 72 => newX = 144 - oldX, heading mirrored => theta = pi - theta
        pathShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 48, 9.85, Math.PI - Math.toRadians(90)),
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115))))
                .build();

        pathToFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115)),
                        new Pose(144 - 49, 25, Math.PI - Math.toRadians(-25))))
                .build();

        pathBackFirstRow = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 49, 25, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 3, 25, Math.PI - Math.toRadians(0))))
                .build();

        pathShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144 - 3, 24, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 20.31, 16.314, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115))))
                .build();

        pathToSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115)),
                        new Pose(144 - 49, 49.125, Math.PI - Math.toRadians(0))))
                .build();

        pathBackSecondRow = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 49, 49.125, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 3, 49.125, Math.PI - Math.toRadians(0))))
                .build();

        pathShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(144 - 3, 49.125, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 32.296, 19.81, Math.PI - Math.toRadians(0)),
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115))))
                .build();

        pathExit = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(144 - 55, 20, Math.PI - Math.toRadians(115)),
                        new Pose(144 - 33, 25, Math.PI - Math.toRadians(180))))
                .build();
    }

    private void runAutonomousStateMachine() {
        switch (pathState) {

            case 1:
                follower.followPath(pathShoot1);
                pathState = 100;
                break;

            case 100:
                if (!follower.isBusy()) pathState = 101;
                break;

            case 101:
                shootMultiple(3);
                pathState = 2;
                break;

            case 2:
                follower.followPath(pathToSecondRow);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pathBackSecondRow);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) pathState = 5;
                break;

            case 5:
                follower.followPath(pathShoot2);
                pathState = 6;
                break;

            case 6:
                if (!follower.isBusy()) {
                    shootMultiple(3);
                    pathState = 7;
                }
                break;

            case 7:
                follower.followPath(pathToFirstRow);
                pathState = 8;
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pathBackFirstRow);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) pathState = 10;
                break;

            case 10:
                follower.followPath(pathShoot3);
                pathState = 11;
                break;

            case 11:
                if (!follower.isBusy()) {
                    shootMultiple(3);
                    pathState = 12;
                }
                break;

            case 12:
                follower.followPath(pathExit);
                pathState = 13;
                break;

            case 13:
                if (!follower.isBusy()) pathState = 14;
                break;
        }
    }

    private void shootMultiple(int count) {
        for (int i = 0; i < count; i++) {
            shoot();
            sleep(100);
        }
    }

    private void shoot() {
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
