package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Default values are for Blue 2
public class AutoCoordinateMap {
    public Pose startPose = new Pose(56.500, 11.500, Math.toRadians(90));
    public Pose shootPose = new Pose(58.000, 21.000, Math.toRadians(110));
    public Pose firstRowStartPose = new Pose(44.000, 87.000, Math.toRadians(0));
    public  Pose firstRowEndPose = new Pose(25.000, 87.000, Math.toRadians(0));
    public  Pose secondRowStartPose = new Pose(46.000, 60.000, Math.toRadians(0));
    public  Pose secondRowEndPose = new Pose(15, 60.000, Math.toRadians(0));
    public  Pose thirdRowStartPose = new Pose(46.000, 38.000, Math.toRadians(0));
    public  Pose thirdRowEndPose = new Pose(15, 38.000, Math.toRadians(0));
    public  Pose gatePoseHalf = new Pose(24, 67, Math.toRadians(90));
    public  Pose gatePoseFinal = new Pose(16, 67, Math.toRadians(90));
    public  Pose endPose = new Pose(38.71408250355619, 33.5931721194879, Math.toRadians(180));
    public Pose homeBaseStartPose = new Pose(20, 38, Math.toRadians(70));
    public Pose homeBaseEndPose = new Pose(5, 13, Math.toRadians(90));



    public AutoCoordinateMap(Pose start_pose, Pose shoot_pose,
                             Pose first_row_sp, Pose first_row_ep,
                             Pose second_row_sp, Pose second_row_ep,
                             Pose third_row_sp, Pose third_row_ep,
                             Pose home_base_sp, Pose home_base_ep,
                             Pose gate_pose_half, Pose gate_pose_final, Pose end) {
        this.startPose = start_pose;
        this.shootPose = shoot_pose;
        this.firstRowStartPose = first_row_sp;
        this.firstRowEndPose = first_row_ep;
        this.secondRowStartPose = second_row_sp;
        this.secondRowEndPose = second_row_ep;
        this.thirdRowStartPose = third_row_sp;
        this.thirdRowEndPose = third_row_ep;
        this.gatePoseHalf = gate_pose_half;
        this.gatePoseFinal = gate_pose_final;
        this.endPose = end;
        this.homeBaseStartPose = home_base_sp;
        this.homeBaseEndPose = home_base_ep;
    }
}
