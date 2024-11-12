package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeepsamvith {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13, -35, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(-34,-35), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(-36, -26), Math.toRadians(180)), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(54.6, 52.7), Math.toRadians(-45))


                .splineToLinearHeading(new Pose2d(new Vector2d(-47, -47), Math.toRadians(225)), Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}