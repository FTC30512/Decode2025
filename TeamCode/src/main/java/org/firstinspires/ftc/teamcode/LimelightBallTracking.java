package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "LimelightBallTracking")
public class LimelightBallTracking extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // Your color detection pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                
                // Horizontal and vertical offset
                telemetry.addData("TX", result.getTx());
                telemetry.addData("TY", result.getTy());

                // Target area
                telemetry.addData("TA", result.getTa());
                telemetry.addData("Distance", calculateDistance(result.getTa()));

            } else {
                telemetry.addData("Valid Target", false);

            }

            telemetry.update();
        }

        limelight.stop();
    }
    public static double calculateDistance(double value) {
        return 25.6 * Math.pow(value, -0.49);
    }
}