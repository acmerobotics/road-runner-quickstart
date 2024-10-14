package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBucketSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                .setTangent(Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(175))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, -42, Math.toRadians(60)), Math.toRadians(90))
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, -42, Math.toRadians(100)), Math.toRadians(90))
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55, -40, 3 * Math.PI/4), Math.toRadians(90))
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-90))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(-160))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-160))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(-160))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.PI/4), Math.toRadians(-160))
                .waitSeconds(0.75)

                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(new Pose2d(-28, -9, 0), Math.toRadians(20))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
