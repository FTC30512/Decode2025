package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

class AutoRotationalPIDTuner extends OpMode{
    @Override
    public void init(){
        follower.setStartingPose(new Pose(72, 72));
    }
    @Override
    public void loop(){
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }  if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();
    }
    private final ArrayList<Double> accelerations = new ArrayList<Double>();
    public static double VELOCITY = 30;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;

    @Override
    public void init_loop(){
        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }
    @Override
    public void start() {
        // Automatically command rotation to 90 degrees (Math.toRadians(90))
        // This triggers the Heading PIDF controllers immediately
        follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();
        follower.setTeleOpDrive(0,0,1,true);

    }
}

