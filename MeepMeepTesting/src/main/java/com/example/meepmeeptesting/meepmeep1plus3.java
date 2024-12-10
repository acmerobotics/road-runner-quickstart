package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmeep1plus3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -61, Math.toRadians(90)))
                .waitSeconds(0.5)

                .strafeTo(new Vector2d(10, -33))
                //put arm up while strafing
                //stop at (10, -34) and place the sample on the bar
                .waitSeconds(0.5)
//                .afterTime(0, shoulder.autonDownHC())
//                .afterTime(0, claw.autonOpenClaw())
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, -34))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(20,-55), Math.toRadians(-90))
                .strafeTo(new Vector2d(35,-44))
                .strafeTo(new Vector2d(35,-5))
                .strafeTo(new Vector2d(45,-5))
                .strafeTo(new Vector2d(45,-53))
                .strafeTo(new Vector2d(45,-5))
                .strafeTo(new Vector2d(55,-5))
                .strafeTo(new Vector2d(55,-53))
                .strafeTo(new Vector2d(55,-5))
                .strafeTo(new Vector2d(61,-5))
                .strafeTo(new Vector2d(61,-53))
                .strafeToConstantHeading(new Vector2d(60,-56))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}