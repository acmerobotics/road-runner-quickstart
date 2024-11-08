package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                .strafeTo(new Vector2d(10, -34))
                //put arm up while strafing
                //stop at (10, -34) and place the sample on the bar
                .lineToY(-40)
                .strafeToSplineHeading(new Vector2d(39, -37), Math.toRadians(45))
                //move arm down to gathering position while strafetospline
                .strafeTo(new Vector2d(44, -32))
                //run intake while strafing to point
                //end intake after meeting the point
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, -55, Math.toRadians(200)), Math.toRadians(200))
                .setReversed(false)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}