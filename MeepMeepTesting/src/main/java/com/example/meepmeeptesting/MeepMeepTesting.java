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

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11, -61, Math.toRadians(90)))
//                        .strafeToLinearHeading(new Vector2d(60, -61), Math.toRadians(90))
//                                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36,-61, Math.toRadians(90)))
//                .strafeToLinearHeading(new Vector2d(-36, -10), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-26, -10), Math.toRadians(0))
//                .build());
        DriveShim drive = myBot.getDrive();
        myBot.runAction(drive.actionBuilder(new Pose2d(-38,-60, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-38, -56), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(180+45))
                .strafeToLinearHeading(new Vector2d(-29, -31), Math.toRadians(161))

//                .strafeToLinearHeading(new Vector2d(-36, -24), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(180+45))
//

//                .strafeToLinearHeading(new Vector2d(-44, -24), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-46, -24), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(180+45))
//
//
//                .strafeToLinearHeading(new Vector2d(-54, -24), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-56, -24), Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(180+45))
//



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
