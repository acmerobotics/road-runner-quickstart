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
                //put arm up while strafing
//                .afterTime(0, viper.autonDown())
//                .afterTime(0, shoulder.autonHC())
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(10, -34))
                //put arm up while strafing
                //stop at (10, -34) and place the sample on the bar
                .waitSeconds(0.5)
//                .afterTime(0, shoulder.autonDownHC())

                .strafeTo(new Vector2d(10, -45))
                .strafeToLinearHeading(new Vector2d(35,-40), Math.toRadians(-90))


                .strafeTo(new Vector2d(35, -13))


                .strafeTo(new Vector2d(45,-13))
                .strafeTo(new Vector2d(45,-53))
                //one in observation zone
                .strafeTo(new Vector2d(45,-13))
                .strafeTo(new Vector2d(55,-13))
                .strafeTo(new Vector2d(55,-53))
                //two in observation zone
                //prepare to grab
                //.afterTime(0, claw.autonNormalPivot())
                .strafeTo(new Vector2d(47,-48))
                .strafeTo(new Vector2d(47,-53))
                //grab sample, routing towards chamber.
                //raise arm to clip
                .strafeToSplineHeading(new Vector2d(10,-34), Math.toRadians(90))
                //clip, routing to push final sample and grab specimen

                .strafeTo(new Vector2d(10, -38))
                .strafeToLinearHeading(new Vector2d(35,-40), Math.toRadians(-90))

                .splineToLinearHeading(new Pose2d(61,-13, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeTo(new Vector2d(61,-53))

                .strafeTo(new Vector2d(47,-48))
                .strafeTo(new Vector2d(47,-53))
                //grab sample, routing towards chamber.
                //raise arm to clip
                .strafeToSplineHeading(new Vector2d(10,-34), Math.toRadians(90))

                .strafeToConstantHeading(new Vector2d(60,-56))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}