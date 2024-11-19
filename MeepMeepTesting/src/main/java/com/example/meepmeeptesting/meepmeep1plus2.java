package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep1plus2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14, -61, Math.toRadians(90)))
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(-10, -33))
                //put arm up while strafing
                //stop at (10, -33) and place the sample on the bar
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-10,-40))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-30,-40))
                .splineToLinearHeading(new Pose2d(new Vector2d(-36, -25), Math.toRadians(180)), Math.toRadians(0))
                .strafeTo(new Vector2d(-34, -25))
                //pick sample
                .strafeTo(new Vector2d(-36, -25))
                .waitSeconds(1)
                .strafeTo(new Vector2d(-38, -34))
                .splineToLinearHeading(new Pose2d(new Vector2d(-54.5, -54.5), Math.toRadians(225)), Math.toRadians(-90))
                .waitSeconds(2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}