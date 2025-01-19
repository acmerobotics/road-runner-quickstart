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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -62, Math.toRadians(90)))
                        // Specimen Hang
                .strafeTo(new Vector2d(-10, -33))
                .waitSeconds(2)

                        // Right Block Pickup AND Drop
                .splineToLinearHeading(new Pose2d(-10, -42, Math.toRadians(270)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(270)), Math.toRadians(70))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(-180))
                .waitSeconds(3.5)

                        // Middle Block Pickup AND Drop
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(270)), Math.toRadians(-180))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(-180))
                .waitSeconds(3.5)

                        // Left Block Pickup AND Drop
                .splineToLinearHeading(new Pose2d(-56, -44, Math.toRadians(310)), Math.toRadians(-180))
                .waitSeconds(2)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(180))

                .build());


        //RED SIDE 4 HANG
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62, Math.toRadians(90)))
//                // Specimen Hang 1
//                .strafeTo(new Vector2d(10, -33))
//
//                // Left Block PUSH
//                .splineToLinearHeading(new Pose2d(22, -38, Math.toRadians(90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(36, -26, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(36, -6, Math.toRadians(90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(42, -48, Math.toRadians(90)), Math.toRadians(90))
//
////                 Middle Block PUSH
//                .splineToLinearHeading(new Pose2d(50, -13, Math.toRadians(90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(54, -50, Math.toRadians(90)), Math.toRadians(90))
//
////                 Right Block PUSH
//                .splineToLinearHeading(new Pose2d(57, -10, Math.toRadians(90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(61, -25, Math.toRadians(90)), Math.toRadians(100000000))
//                .splineToLinearHeading(new Pose2d(61, -55, Math.toRadians(90)), Math.toRadians(100000000))
//
//
////                 HANG 2
//                .splineToLinearHeading(new Pose2d(42, -63, Math.toRadians(90)), Math.toRadians(100000000))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(8, -33))
//                .waitSeconds(1)
//
////                 HANG 3
//                .strafeTo(new Vector2d(42, -63))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(4, -33))
//                .waitSeconds(1)
//
////                 HANG 4
//                .strafeTo(new Vector2d(42, -63))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(0, -33))
//
//                .build());

//BLUE SIDE 3 + 1
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(270)))
//                // Specimen Hang
//                .strafeTo(new Vector2d(10, 33))
//                .waitSeconds(2)
//
//                // Right Block Pickup AND Drop
//                .splineToLinearHeading(new Pose2d(10, 42, Math.toRadians(90)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(90)), Math.toRadians(70))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(90))
//                .waitSeconds(3.5)
//
//                // Middle Block Pickup AND Drop
//                .splineToLinearHeading(new Pose2d(58, 48, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(90))
//                .waitSeconds(3.5)
//
//                // Left Block Pickup AND Drop
//                .splineToLinearHeading(new Pose2d(56, 44, Math.toRadians(125)), Math.toRadians(90))
//                .waitSeconds(2)
//                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(90))
//
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
