package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .forwardZeroPowerAcceleration(-35.76327079861690333)
            .lateralZeroPowerAcceleration(-52.565504041356866667)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.052, 0, 0.0001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.55, 0, 0, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0, 0.6, 0.02))
            .centripetalScaling(0.0004)
//            .forwardZeroPowerAcceleration(-32.724856224412534)
//            .lateralZeroPowerAcceleration(-49.8268941546)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0, 0.02))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.00005, 0, 0.02))
//            .useSecondaryTranslationalPIDF(true)
//            .headingPIDFCoefficients(new PIDFCoefficients(0.55, 0, 0, 0.02))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.02))
//            .centripetalScaling(0.0005)
            ;

    public static PathConstraints pathConstraints = new PathConstraints
            (0.99, 100, 1.5, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.7)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(30.520888200895056333333333333333)
            .yVelocity(23.572448730468751666666666666667)
//            .xVelocity(65.828086306404763333333333333333)
//            .yVelocity(52.215)
            ;

//    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
////            .forwardTicksToInches(0.00364871156560783960469237135065)
////            .strafeTicksToInches(0.00196852125071119356833642547928)
////            .turnTicksToInches(.001989436789)
//            .forwardTicksToInches(.001989436789)
//            .strafeTicksToInches(.001989436789)
//            //.turnTicksToInches(.001989436789)
//            //.leftPodY(5.25)
//            //.rightPodY(-5.25)
//            .forwardPodY(-5.25)
//            .strafePodX(5.25)
//            //.leftEncoder_HardwareMapName("leftFront")
//            //.rightEncoder_HardwareMapName("rightRear")
//            .forwardEncoder_HardwareMapName("leftFront")
//            .strafeEncoder_HardwareMapName("rightRear")
//            //.leftEncoderDirection(Encoder.REVERSE)
//            //.rightEncoderDirection(Encoder.REVERSE)
//            .forwardEncoderDirection(Encoder.REVERSE)
//            .strafeEncoderDirection(Encoder.FORWARD)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(
//                    new RevHubOrientationOnRobot(
//                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
//                    )
//            )
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.25)
            .strafePodX(5.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}