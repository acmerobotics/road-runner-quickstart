package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                .setTangent(3 * Math.PI/4)
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), Math.PI)
                .waitSeconds(1)

                .setTangent(Math.PI/4)
                .splineToLinearHeading(new Pose2d(-36, -36, 3 * Math.PI/4), Math.PI/4)
                .waitSeconds(1)
                .setTangent(5 * Math.PI/4)
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), 5 * Math.PI/4)
                .waitSeconds(1)

                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(-50, -36, 3 * Math.PI/4), Math.toRadians(100))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-80))
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), Math.toRadians(-80))
                .waitSeconds(1)

                .setTangent(3 * Math.PI/4)
                .splineToLinearHeading(new Pose2d(-58, -36, 3 * Math.PI/4), 3 * Math.PI/4)
                .waitSeconds(1)
                .setTangent(Math.PI/-4)
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), Math.PI/-4)
                .waitSeconds(1)

                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-28, -9, 0), 0)
                .waitSeconds(1)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), -Math.PI/2)
                .waitSeconds(1)

                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-28, -9, 0), 0)
                .waitSeconds(1)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(-47, -47, Math.PI/4), -Math.PI/2)
                .waitSeconds(1)

                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(-28, -9, Math.PI), 0)
//                Observation Side Auton (sid)
//                .splineToLinearHeading(new Pose2d(58.0, -50.0, Math.toRadians(90)), Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(58.0, -40.0, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(58.0, -50.0, Math.toRadians(90)), Math.toRadians(270))
//                .waitSeconds(1)
//
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(53.0, -40, Math.toRadians(120)), Math.toRadians(90))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(58.0, -50.0, Math.toRadians(90)), Math.toRadians(270))
//                .waitSeconds(1)
//
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(53.0, -40, Math.toRadians(45)), Math.toRadians(90))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(58.0, -50.0, Math.toRadians(90)), Math.toRadians(270))
//                .waitSeconds(1)
//
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(35.0, -35.0, Math.toRadians(90)), Math.toRadians(90))
//                .waitSeconds(12)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(35.0, -60.0, Math.toRadians(90)), Math.toRadians(270))


//                CYCLES
//                .setReversed(false)
//                .splineTo(new Vector2d(30, -12), Math.toRadians(180))
//                .waitSeconds(1.8)
//                .setReversed(true)
//                .splineTo(new Vector2d(58.0, -50.0), Math.toRadians(270))
//
//                .setReversed(false)
//                .splineTo(new Vector2d(30, -12), Math.toRadians(180))
//                .waitSeconds(1.8)
//                .setReversed(true)
//                .splineTo(new Vector2d(58.0, -50.0), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
