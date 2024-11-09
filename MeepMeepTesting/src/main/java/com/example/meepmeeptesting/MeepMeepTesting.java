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
                .setReversed(true)
                .splineTo(new Vector2d(30, -36), Math.toRadians(0))
                //move arm down to gathering position while splining
                .splineTo(new Vector2d(35, -5), Math.toRadians(90))
                        .setReversed(false)

                .splineToConstantHeading(new Vector2d(48, -20), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(48, -50), Math.toRadians(-90))


                //run intake while strafing to point
                //end intake after meeting the point

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}