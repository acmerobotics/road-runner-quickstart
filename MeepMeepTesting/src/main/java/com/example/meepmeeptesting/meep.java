package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meep {

    public static double startx = -35.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -25.0;
    public static double scoreHubPosy = 38.0;
    public static double scoreHubPosAng = Math.toRadians(-40);

    public static double carouselPosx = -60.0;
    public static double carouselPosy = 56.0;
    public static double carouselPosAng = Math.toRadians(180);

    public static double parkX = -60.0;
    public static double parkY = 35.0;
    public static double parkAng = Math.toRadians(180);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        Pose2d startPos = new Pose2d(startx,starty, startAng);
        Vector2d scoreHubPos = new Vector2d(scoreHubPosx,scoreHubPosy);
        Pose2d carouselPos = new Pose2d(carouselPosx,carouselPosy,carouselPosAng);
        Pose2d park = new Pose2d(parkX,parkY,parkAng);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                            .setReversed(true)
                            .splineTo(scoreHubPos,scoreHubPosAng)
                            //slides
                            .lineToSplineHeading(carouselPos)
                            //carousel
                            .lineToSplineHeading(park)
                            .build()

                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}