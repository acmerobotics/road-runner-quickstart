package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 16)
                .build();
//        RED SIDE 3 + 1
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -62, Math.toRadians(270)))
                        // Specimen Hang
                .strafeTo(new Vector2d(-10, -33))
                .waitSeconds(2)

                        // Right Block Pickup AND Drop
                .strafeTo(new Vector2d(-14, -45))
                .splineToLinearHeading(new Pose2d(-35, -25.5, Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-180))
                .waitSeconds(3.5)

                        // Middle Block Pickup AND Drop
                .strafeTo(new Vector2d(-50, -56))
                .splineToLinearHeading(new Pose2d(-45, -25.5, Math.toRadians(180)), Math.toRadians(-270))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-180))
                .waitSeconds(3.5)

//                        // Left Block Pickup AND Drop
                .strafeTo(new Vector2d(-50, -56))
                .splineToLinearHeading(new Pose2d(-55, -25.5, Math.toRadians(180)), Math.toRadians(-180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-54, -25.5))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(45)), Math.toRadians(-180))
                .build());


        //RED SIDE 4 HANG
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62, Math.toRadians(270)))
//                // Specimen Hang 1
//                .strafeTo(new Vector2d(10, -33))
//
//                // Left Block PUSH
//                .strafeTo(new Vector2d(10, -40))
//                .splineTo(new Vector2d(41, -7), Math.toRadians(90))
//                .strafeTo(new Vector2d(48, -12))
//                .strafeTo(new Vector2d(48, -55))

                // Middle Block PUSH
//                .splineToLinearHeading(new Pose2d(58, -10, Math.toRadians(180)), Math.toRadians(-360))
//                .strafeTo(new Vector2d(58, -55))

                // Right Block PUSH
//                .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(180)), Math.toRadians(-360))
//                .strafeTo(new Vector2d(62, -55))

                // HANG 2
//                .splineTo(new Vector2d(48, -63), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(8, -33, Math.toRadians(270)), Math.toRadians(360))

                // HANG 3
//                .splineToLinearHeading(new Pose2d(48, -63, Math.toRadians(90)), Math.toRadians(360))
//                .splineToLinearHeading(new Pose2d(6, -33, Math.toRadians(270)), Math.toRadians(360))

                // HANG 4
//                .splineToLinearHeading(new Pose2d(48, -63, Math.toRadians(90)), Math.toRadians(360))
//                .splineToLinearHeading(new Pose2d(4, -33, Math.toRadians(270)), Math.toRadians(360))

//                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}