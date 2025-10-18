package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunnerSetup.MecanumDrive;

@Autonomous
public class BlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Create your Road Runner drive instance
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-58,-58, Math.toRadians(180)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-58, -58, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(58, -35), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(40, -30, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(35, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(15,-17, Math.toRadians(180)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-22,-21, Math.toRadians(225)), Math.toRadians(225))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(12,-30, Math.toRadians(90)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(12, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(15,-17, Math.toRadians(180)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-22,-21, Math.toRadians(225)), Math.toRadians(225))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(-12,-30, Math.toRadians(90)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-15,-17, Math.toRadians(180)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(-22,-21, Math.toRadians(225)), Math.toRadians(225))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(0,-53, Math.toRadians(90)), Math.toRadians(225))
                        .build());
    }
}
