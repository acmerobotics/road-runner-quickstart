package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingObservationSide {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                .setTangent(Math.toRadians(113))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), Math.toRadians(113))
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(-15))
                .splineToLinearHeading(new Pose2d(60, -50, Math.toRadians(90)), Math.toRadians(-15))
                .waitSeconds(0.5)

                .turn(Math.toRadians(20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(1)

                // gonna see me cycling
                .setTangent(Math.toRadians(165))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(165))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-5))
                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(-90)), Math.toRadians(-5))
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(175))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-5))
                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(-90)), Math.toRadians(-5))
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(175))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-5))
                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(-90)), Math.toRadians(-5))
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(175))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), Math.toRadians(175))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-5))
                .splineToLinearHeading(new Pose2d(36, -40, Math.toRadians(-90)), Math.toRadians(-5))
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(36, -60, Math.toRadians(-90)), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
