package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.Movement;
import org.firstinspires.ftc.teamcode.helpers.PointToAprilTag;
import org.firstinspires.ftc.teamcode.helpers.Velocity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "EW TeleOp")
public class EWTeleOp extends LinearOpMode {

    // Camera calibration (for Logitech C920 @ 800x448)


    @Override
    public void runOpMode() {

        // Define your custom AprilTags
        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)
                .addTag(22, "Motif Pattern", 6.5, DistanceUnit.INCH)
                .build();

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");   // port 0
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");     // port 1
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront"); // port 2
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");   // port 3
        DcMotor shooter = hardwareMap.dcMotor.get("Shooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        Movement movement = new Movement(leftFront, leftRear, rightFront, rightRear, gamepad1);
        PointToAprilTag pointToAprilTag = new PointToAprilTag(hardwareMap, telemetry);
        Velocity velocity = new Velocity(pointToAprilTag);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Regular driving
            movement.drive();

            // --- Update AprilTag each loop ---
            pointToAprilTag.updateTagDistance();

            // --- Turn toward tag if right trigger is pressed ---
            if (gamepad1.right_trigger > 0.5) {

                pointToAprilTag.turnToTag(leftFront, leftRear, rightFront, rightRear);
                shooter.setPower(velocity.getSpeed(hardwareMap, telemetry));

            }

            telemetry.update();
        }
    }
}
