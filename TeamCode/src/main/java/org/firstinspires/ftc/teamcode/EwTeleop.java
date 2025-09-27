package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import android.util.Size;

import java.util.List;

@TeleOp
public class EwTeleop extends LinearOpMode {

    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

    @Override
    public void runOpMode() {

        // Configure the red blob detector
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // Set up camera + processor
        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240)) // Smaller resolution for performance
                .setCamera(hardwareMap.get(WebcamName.class, "EwWebcam"))
                .build();

        telemetry.addLine("Vision ready. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

        }

        visionPortal.close();
    }
}
