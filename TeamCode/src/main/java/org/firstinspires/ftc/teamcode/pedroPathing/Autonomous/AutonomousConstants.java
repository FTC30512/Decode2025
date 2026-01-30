package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.paths.PathChain;

public class AutonomousConstants {
    public enum AutoVariations {
        FIRSTROW,
        SECONDROW,
        THIRDROW,
        SECONDROW_OPEN_GATE,
        ENDPOSE
    }
    AutoVariations[] autoVariations = new AutoVariations[5];

    public enum PathState{
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
    public PathChain
            pathStarttoShoot,
            pathShoottoSecond,
            pathSecondCollect,
            pathSecondtoShoot,
            pathShoottoFirst,
            pathFirstCollect,
            pathFirsttoShoot,
            pathShoottoEnd,
            pathSecondtoGateHalf,
            pathGateHalftoGateFinal,
            pathGateFinaltoShoot;
}
