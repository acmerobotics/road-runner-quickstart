package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
//                .setTangent(Math.toRadians(175))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(175))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-55, -42, Math.toRadians(60)), Math.toRadians(90))
//                .waitSeconds(0.75)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-55, -42, Math.toRadians(100)), Math.toRadians(90))
//                .waitSeconds(0.75)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-55, -40, 3 * Math.PI/4), Math.toRadians(90))
//                .waitSeconds(0.75)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(20))
//                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))
//                .waitSeconds(0.75)
//                .setTangent(Math.toRadians(-160))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-160))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(20))
//                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))
//                .waitSeconds(0.75)
//                .setTangent(Math.toRadians(-160))
//                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-160))
//                .waitSeconds(0.75)
//
//                .setTangent(Math.toRadians(20))
//                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))
//                Observation Side Auton
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), Math.toRadians(110))
                .waitSeconds(0.5)

                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(58, -50, Math.toRadians(90)), -Math.PI/10)
                .waitSeconds(0.5)

                .turn(Math.toRadians(20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(1)

                // gonna see me cycling
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
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
//
//
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
