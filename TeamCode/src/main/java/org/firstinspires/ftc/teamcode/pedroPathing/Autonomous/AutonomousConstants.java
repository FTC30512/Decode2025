package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.paths.PathChain;

public class AutonomousConstants {

    public enum PathState {
        STARTPOS_SHOOTPOS,
        SHOOTPOS_FIRSTROW,
        SHOOTPOS_SECONDROW,
        SHOOTPOS_THIRDROW,
        COLLECT_FIRSTROW,
        COLLECT_SECONDROW,
        SECONDROW_GATEHALF,
        GATEHALF_GATEFINAL,
        GATEFINAL_SHOOT,
        COLLECT_THIRDROW,
        FIRSTROW_SHOOTPOS,
        SECONDROW_SHOOTPOS,
        THIRDROW_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT,
        STOP
    }
    double collectSpeed = 0.65;
    PathState pathState;
    int farShooterSpeed = 2525;
    int nearShooterSpeed = 2300;
    double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
}
