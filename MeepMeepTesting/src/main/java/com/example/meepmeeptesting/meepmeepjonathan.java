package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeepjonathan {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                //.afterTime(0, shoulder.autonHC())
                //.afterTime(0.8, viper.autonHangSpecimen())
                .strafeTo(new Vector2d(7, -32))

                //put arm up while strafing
                //stop at (9, -30) and place the sample on the bar
                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0, viper.autonDown())
                .waitSeconds(0.5)



                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(26,-43), Math.toRadians(-90)), 0)
                //.afterTime(0, shoulder.autonDown())

                .splineTo(new Vector2d(45, -13), 0)

                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))
                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(55,-53))
                //two in observation zone
                //prepare to grab
                .strafeTo(new Vector2d(45,-53))

                .strafeTo(new Vector2d(45,-58))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(45,-59.5))

                //.afterTime(0, viper.autonSlightOut())
                .waitSeconds(0.1)
                //.afterTime(0, claw.autonCloseClaw())
                .waitSeconds(0.5)
                //.afterTime(0, shoulder.autonHC())
                //grab sample, routing towards chamber.
                //raise arm to clip
                //.afterTime(3, viper.autonHangSpecimen())
                        .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(9, -32), Math.toRadians(90)), Math.toRadians(90))

                //clip, routing to push final sample and grab specimen

                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0, viper.autonDown())



                //.afterTime(1, shoulder.autonDown())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(40, -58), Math.toRadians(-90)), Math.toRadians(0))



//                .splineToSplineHeading(new Pose2d(new Vector2d(60,-13), Math.toRadians(-90)), Math.toRadians(-90))
//
//                //.splineToLinearHeading(new Pose2d(61,-13, Math.toRadians(-90)), Math.toRadians(-90))
//                .strafeTo(new Vector2d(60,-53))
//
//                .strafeTo(new Vector2d(45,-53))
//                .strafeTo(new Vector2d(45,-58))
//                .waitSeconds(0.1)
                .strafeTo(new Vector2d(45,-59.5))

                //.afterTime(0.1, claw.autonCloseClaw())
                .waitSeconds(0.5)
                //.afterTime(0, shoulder.autonHC())
                //grab sample, routing towards chamber.
                //raise arm to clip
                //.afterTime(3, viper.autonHangSpecimen())
                        .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(9, -32), Math.toRadians(90)), Math.toRadians(90))

                //.afterTime(0, claw.autonOpenClaw())
                //.afterTime(0, viper.autonDown())
                .waitSeconds(0.5)

                        .setReversed(true)
                .splineTo(new Vector2d(50,-60), Math.toRadians(-90))
                //.afterTime(0, shoulder.autonDown())

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}