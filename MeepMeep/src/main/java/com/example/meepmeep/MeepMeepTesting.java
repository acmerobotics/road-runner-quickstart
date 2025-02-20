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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, 62, Math.toRadians(270)))
//                        .strafeTo(new Vector2d(-36, 62))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-3, 33, Math.toRadians(270)), Math.toRadians(270))
//                        .waitSeconds(.7)
//                        .setTangent(Math.toRadians(180))
//                        .splineToSplineHeading(new Pose2d(-34, 43, Math.toRadians(240)), Math.toRadians(180))
//                        .waitSeconds(.5)
//                        .splineToLinearHeading(new Pose2d(-32, 46, Math.toRadians(135)), Math.toRadians(135))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
