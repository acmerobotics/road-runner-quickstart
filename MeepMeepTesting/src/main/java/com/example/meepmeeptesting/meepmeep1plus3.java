package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep1plus3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                //put arm up while strafing
//                .afterTime(0, viper.autonDown())
//                .afterTime(0, shoulder.autonHC())
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(10, -32))
                //put arm up while strafing
                //stop at (10, -34) and place the sample on the bar
                .waitSeconds(0.5)
//                .afterTime(0, shoulder.autonDownHC())
                .waitSeconds(1)
                .setReversed(true)
                .strafeTo(new Vector2d(10, -36))
                .splineTo(new Vector2d(28, -36), Math.toRadians(0))
                //move arm down to gathering position while splining
//                .afterTime(0, shoulder.autonDown())
                .splineTo(new Vector2d(33, -5), Math.toRadians(90))
                .setReversed(false)

                .splineToConstantHeading(new Vector2d(47, -20), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(47, -50), Math.toRadians(-90))
                //shrey code starts here
                .setReversed(true)
                .strafeTo(new Vector2d(47, -10))


                //.splineToConstantHeading(new Vector2d(52,-45), Math.toRadians
                .strafeTo(new Vector2d(56,-10))

                .strafeTo(new Vector2d(56,-53))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(60,-56))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}