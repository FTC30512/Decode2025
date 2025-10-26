package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Create the MeepMeep simulator window
        MeepMeep meepMeep = new MeepMeep(800);

        // Build a simulated Road Runner robot
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Robot motion constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                .setDimensions(17,  17.5)
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(new Pose2d(61, -28.0, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(43,-28, Math.toRadians(90)), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(35, -50), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(53, -8, Math.toRadians(215)), Math.toRadians(90))
                        .build()
        );

        // Visual configuration
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
