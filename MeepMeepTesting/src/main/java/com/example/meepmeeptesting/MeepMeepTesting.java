package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double startx = -12;
        double starty = -72;
        double heading = 270;


        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(heading));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(startPos)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                /*.back(10)
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo(new Vector2d(-12, -36), Math.toRadians(90))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineTo(new Vector2d(11, -65), Math.toRadians(-20))
                                .splineTo(new Vector2d(40, -62), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(25, -65), Math.toRadians(200))
                                .splineTo(new Vector2d(-12, -36), Math.toRadians(90))
                                .waitSeconds(1)*/
                                .lineTo(new Vector2d(-12,-42))

                                .splineTo(new Vector2d(-48,-48),Math.toRadians(90))
                                //do stuff with high goal

                                .build()

                );




        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}