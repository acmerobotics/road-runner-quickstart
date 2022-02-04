package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double startx = 0;
        double starty = -72;



        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
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
                                .back(5)
                                .build()

                );

        RoadRunnerBotEntity a = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder((new Pose2d(-47, -62, Math.toRadians(0))))
                                .back(5)
                                .waitSeconds(2)
                                //.turn(Math.toRadians(180))


                                .splineTo(new Vector2d(-12, -36), Math.toRadians(-90))
                                .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}