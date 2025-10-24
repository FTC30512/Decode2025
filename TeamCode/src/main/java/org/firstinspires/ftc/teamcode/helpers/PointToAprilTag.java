package org.firstinspires.ftc.teamcode.helpers;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class PointToAprilTag {
    private final AprilTagProcessor aprilTagProcessor;
    private final Telemetry telemetry;

    // 👇 Stores the most recent distance (inches)
    public double lastRangeIN = 0;

    // Camera calibration (for Logitech C920 @ 800x448)
    public double fx = 477.2;
    public double fy = 474.6;
    public double cx = 399.6;
    public double cy = 224.0;

    public PointToAprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Define your custom AprilTags
        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagLibrary(aprilTagLibrary)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .setCameraResolution(new Size(800, 448))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // 👇 Updates distance (inches)
    public void updateTagDistance() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            if (tag != null){
                lastRangeIN = tag.ftcPose.range;

            }
        }
    }

    // 👇 Turns robot to face the detected AprilTag
    @SuppressLint("DefaultLocale")
    public void turnToTag(DcMotor lf, DcMotor lr, DcMotor rf, DcMotor rr) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);

            // Update last distance (inches)
            lastRangeIN = tag.ftcPose.range;

//            telemetry.addLine("Found AprilTag!");
//            telemetry.addData("Tag ID", tag.id);
//            telemetry.addData("Distance (in)", lastRangeIN);
//            telemetry.addData("Yaw (deg)", "%.1f", tag.ftcPose.yaw);

            double xError = tag.ftcPose.x;
            double turnPower = xError * 0.5;
            turnPower = Math.max(-1.0, Math.min(turnPower, 1.0));


            if (xError > 0) {
                lf.setPower(turnPower);
                lr.setPower(turnPower);
                rf.setPower(-turnPower);
                rr.setPower(-turnPower);
            }else {
                lf.setPower(-turnPower);
                lr.setPower(-turnPower);
                rf.setPower(turnPower);
                rr.setPower(turnPower);
            }

            // Stop turning when aligned
            if (Math.abs(xError) < 5.0) {
                lf.setPower(0);
                lr.setPower(0);
                rf.setPower(0);
                rr.setPower(0);
                return;
            }

        } else {
            telemetry.addLine("No AprilTags detected");
            lf.setPower(0);
            lr.setPower(0);
            rf.setPower(0);
            rr.setPower(0);
        }

        telemetry.update();
    }
}
