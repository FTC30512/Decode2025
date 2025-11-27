package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunnerSetup.MecanumDrive;
import org.jetbrains.annotations.NotNull;

@Autonomous
public class BlueAuto extends LinearOpMode {
    private Servo gateServo, shooterServo;

    // --- Motors ---
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor intake, shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        // Create your Road Runner drive instance
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-58,-58, Math.toRadians(180)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-58, -58, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(43,-28, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(35, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(53, -8, Math.toRadians(215)), Math.toRadians(90))
                        .stopAndAdd(new shoot(shooter, gateServo, shooterServo))
                        .splineToSplineHeading(new Pose2d(20,-28, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(12, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(53, -8, Math.toRadians(215)), Math.toRadians(90))
                        .stopAndAdd(new shoot(shooter, gateServo, shooterServo))
                        .build()
        );
    }

    public class shoot implements Action {
        Servo gateServo, shooterServo;
        DcMotor shooter;
        double speed;

        public shoot(DcMotor shooter, Servo gateServo, Servo shooterServo){
            this.shooter = shooter;
            this.gateServo = gateServo;
            this.shooterServo = shooterServo;
        }

        @Override
        public boolean run(@NotNull TelemetryPacket telemetryPacket) {
            shooter.setPower(1);
            while (shooter.getPower() != 1){

            }
            for (int i = 0; i < 3; i++) {
                gateServo.setPosition(0.3);
                sleep(200);
                shooterServo.setPosition(1);
                sleep(500);
                shooterServo.setPosition(0.65);
                sleep(350);
                gateServo.setPosition(0);
                sleep(200);
            }
            return false;
        }
    }
    private void initHardware() {
        // Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        intake = hardwareMap.dcMotor.get("Intake");
        shooter = hardwareMap.dcMotor.get("Shooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Servos
        shooterServo = hardwareMap.servo.get("shooterServo");
        gateServo = hardwareMap.servo.get("gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.REVERSE);
    }
}
