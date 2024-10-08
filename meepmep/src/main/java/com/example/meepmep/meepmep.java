package com.example.meepmep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(11,58))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(56,48), Math.toRadians(245))
                .waitSeconds(3)
                .turn(Math.toRadians(30))
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(55))
                .waitSeconds(1)
                .turn(Math.toRadians(-55))
                .waitSeconds(2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}