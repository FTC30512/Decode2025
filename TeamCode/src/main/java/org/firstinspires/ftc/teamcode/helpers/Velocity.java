package org.firstinspires.ftc.teamcode.helpers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "Launcher Velocity (inches)")
public class Velocity {

    double g = 385.827;             // gravity in inches/s^2
    double targetHeight = 46.0;   // target height (in)
    double launcherHeight = 15.0;   // launcher height (in)
    double launchAngleDeg = 45.0;  // launch angle (degrees)
    double cameraHeight = 8.0;
    double aprilTagHeight = 29.5;
    double distanceToTarget;
    public double motorPower;
    PointToAprilTag pointToAprilTag;

    public Velocity(PointToAprilTag l_pointToAprilTag){
        pointToAprilTag = l_pointToAprilTag;
    }
    //PointToAprilTag pointToAprilTag = new PointToAprilTag(hardwareMap, telemetry);
    MMtoInch mMtoInch = new MMtoInch();

    public double getSpeed(HardwareMap hardwareMap, Telemetry telemetry){

        pointToAprilTag.updateTagDistance();


        if (pointToAprilTag.lastRangeIN > 0) {
            double distanceToAprilTag = pointToAprilTag.lastRangeIN;

            double distanceToTarget2 = (Math.pow(distanceToAprilTag, 2) - Math.pow(aprilTagHeight - cameraHeight, 2));

            if (distanceToTarget2<0){
                telemetry.addLine("The distanceToAprilTag is wrong");
                return 0;
            }else {
                distanceToTarget = Math.sqrt(distanceToTarget2);
            }

            double angleRad = Math.toRadians(launchAngleDeg);
            double deltaH = targetHeight - launcherHeight;

            double numerator = g * Math.pow(distanceToTarget, 2);
            if ((distanceToTarget * Math.tan(angleRad) - deltaH) <= 0){
                telemetry.addLine("Too Close");
                return 0;
            }
            double denominator = 2 * Math.pow(Math.cos(angleRad), 2) * (distanceToTarget * Math.tan(angleRad) - deltaH);

            if (denominator > 0) {
                double v = Math.sqrt(numerator / denominator); // in/s

                double wheelDiameter = mMtoInch.converter(72);
                double maxRPM = 6000.0;

                double wheelRPM = (v / (Math.PI * wheelDiameter)) * 60.0;
                motorPower = Math.max(0, Math.min(1, wheelRPM / maxRPM));

                telemetry.addData("Required Velocity (in/s)", v);
                telemetry.addData("Wheel RPM", wheelRPM);
                telemetry.addData("Motor Power", motorPower);
                telemetry.addData("Distance", distanceToAprilTag);
            } else {
                telemetry.addLine("Invalid geometry for this distance/angle");
            }
        } else {
            telemetry.addLine("Waiting for AprilTag...");
        }

        telemetry.update();
        return motorPower * 1.5;
    }
}
