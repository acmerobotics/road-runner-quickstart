package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meep {
    public static double tuningNumber = 36;
    public static double tuningTimer = 1;


    public static double startx = 0;
    public static double starty = -72;

    public static double bankcurveX = -5;
    public static double bankcurveY = starty + 22;
    public static int cycles = 4;

    public static double maxVel = 40;



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        Pose2d startPos = new Pose2d(startx-tuningNumber,starty, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.85)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                            .forward(10)
                            .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}