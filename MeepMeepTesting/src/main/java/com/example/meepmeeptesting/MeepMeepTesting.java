package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        DriveShim drive = myBot.getDrive();
        //near basket
//        myBot.runAction(drive.actionBuilder(new Pose2d(-38,-60, Math.toRadians(180)))
//                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
//                .strafeToLinearHeading(new Vector2d(-48, -43), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
//                .strafeToLinearHeading(new Vector2d(-60, -40), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
//                .strafeToLinearHeading(new Vector2d(-60, -24), Math.toRadians(180))


        // near observation zone
        myBot.runAction(drive.actionBuilder(new Pose2d(9,-60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(9, -34), Math.toRadians(90))








                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
