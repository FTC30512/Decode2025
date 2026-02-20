package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutonomousConstants {

    public enum PathState {
        STARTPOS_SHOOTPOS,
        SHOOTPOS_FIRSTROW,
        SHOOTPOS_SECONDROW,
        SHOOTPOS_THIRDROW,
        SHOOTPOS_HOME,
        COLLECT_FIRSTROW,
        COLLECT_SECONDROW,
        COLLECT_THIRDROW,
        COLLECT_HOME,
        SECONDROW_GATEHALF,
        GATEHALF_GATEFINAL,
        GATEFINAL_SHOOT,
        FIRSTROW_SHOOTPOS,
        SECONDROW_SHOOTPOS,
        THIRDROW_SHOOTPOS,
        HOME_SHOOTPOS,
        SHOOTPOS_ENDPOSE,
        SHOOT,
        STOP
    }
    double collectSpeed = 0.65;
    PathState pathState;
    int farShooterSpeed = 2525;
    int nearShooterSpeed = 2300;
    double Kp = 255.0, Ki = 0.0, Kd = 0.0, Kf = 11.62;
    AutoCoordinateMap AutoBlue1 = new AutoCoordinateMap(
            new Pose(19.6, 126.6, Math.toRadians(140.3)),  // Start Pose
            new Pose(57.000, 86.000, Math.toRadians(135)), // Shoot Pose
            new Pose(52, 87.000, Math.toRadians(0)),       // First Row Start Pose
            new Pose(20, 87.000, Math.toRadians(0)),       // First Row End Pose
            new Pose(48, 63.500, Math.toRadians(0)),       // Second Row Start Pose
            new Pose(13, 63.500, Math.toRadians(0)),       // Second Row End Pose
            new Pose(44.000, 38.000, Math.toRadians(0)),   // Third Row Start Pose
            new Pose(22.000, 38.000, Math.toRadians(0)),   // Third Row End Pose
            new Pose(20, 38, Math.toRadians(70)),          // Home Base Start Pose
            new Pose(7.5, 10, Math.toRadians(90)),         // Home Base End Pose
            new Pose(26, 71.5, Math.toRadians(90)),        // Gate Pose Half
            new Pose(16, 71.5, Math.toRadians(90)),        // Gate Pose Final
            new Pose(48, 72, Math.toRadians(180))          // End Pose
    );
    AutoCoordinateMap AutoBlue2 = new AutoCoordinateMap(
            new Pose(56.500, 11.500, Math.toRadians(90)),                         // Start Pose
            new Pose(58.000, 21.000, Math.toRadians(105)),                        // Shoot Pose
            new Pose(44.000, 87.000, Math.toRadians(0)),                          // First Row Start Pose
            new Pose(25.000, 87.000, Math.toRadians(0)),                          // First Row End Pose
            new Pose(46.000, 60.000, Math.toRadians(0)),                          // Second Row Start Pose
            new Pose(15, 60.000, Math.toRadians(0)),                              // Second Row End Pose
            new Pose(46.000, 38.000, Math.toRadians(0)),                          // Third Row Start Pose
            new Pose(15, 38.000, Math.toRadians(0)),                              // Third Row End Pose
            new Pose(24, 38, Math.toRadians(50)),                                 // Home Base Start Pose
            new Pose(8, 13, Math.toRadians(90)),                                // Home Base End Pose
            new Pose(24, 67, Math.toRadians(90)),                                 // Gate Pose Half
            new Pose(16, 67, Math.toRadians(90)),                                 // Gate Pose Final
            new Pose(38.71408250355619, 33.5931721194879, Math.toRadians(180))    // End Pose
    );
    AutoCoordinateMap AutoRed1 = new AutoCoordinateMap(
            new Pose(124.8, 126.8, Math.toRadians(43.8)),   // Start Pose
            new Pose(90, 86.000, Math.toRadians(45)),       // Shoot Pose
            new Pose(98.000, 88.000, Math.toRadians(180)),  // First Row Start Pose
            new Pose(125, 88.000, Math.toRadians(180)),     // First Row End Pose
            new Pose(98.000, 64.000, Math.toRadians(180)),  // Second Row Start Pose
            new Pose(135, 64.000, Math.toRadians(180)),     // Second Row End Pose
            new Pose(100.000, 39.000, Math.toRadians(180)), // Third Row Start Pose
            new Pose(120.000, 39.000, Math.toRadians(180)), // Third Row End Pose
            new Pose(124, 38, Math.toRadians(70)),          // Home Base Start Pose
            new Pose(136.5, 10, Math.toRadians(90)),        // Home Base End Pose
            new Pose(120, 71.5, Math.toRadians(90)),        // Gate Pose Half
            new Pose(129, 71.5, Math.toRadians(90)),        // Gate Pose Final
            new Pose(96, 72, Math.toRadians(0))             // End Pose
    );
    AutoCoordinateMap AutoRed2 = new AutoCoordinateMap(
            new Pose(87.5, 11.5, Math.toRadians(90)),                              // Start Pose
            new Pose(86.000, 21.000, Math.toRadians(64)),                          // Shoot Pose
            new Pose(98.000, 87.000, Math.toRadians(180)),                         // First Row Start Pose
            new Pose(120.000, 87, Math.toRadians(180)),                            // First Row End Pose
            new Pose(98.000, 60.000, Math.toRadians(180)),                         // Second Row Start Pose
            new Pose(129.000, 60.000, Math.toRadians(180)),                        // Second Row End Pose
            new Pose(98.000, 38.000, Math.toRadians(180)),                         // Third Row Start Pose
            new Pose(129.000, 38.000, Math.toRadians(180)),                        // Third Row End Pose
            new Pose(116, 38, Math.toRadians(130)),                                 // Home Base Start Pose
            new Pose(136.5, 13, Math.toRadians(90)),                               // Home Base End Pose
            new Pose(120, 67, Math.toRadians(90)),                                 // Gate Pose Half
            new Pose(128, 67, Math.toRadians(90)),                                 // Gate Pose Final
            new Pose(105.32586660900631, 33.079429420265015, Math.toRadians(0))    // End Pose
    );


}
