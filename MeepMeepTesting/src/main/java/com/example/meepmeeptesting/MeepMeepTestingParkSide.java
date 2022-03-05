package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class MeepMeepTestingParkSide {

    public static double startx = -36.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 40;

    public static double scoreHubPosAngB = -25;
    public static double scoreHubPosAngR = 25;

    public static double carouselPosx = -62;
    public static double carouselPosy = 64;
    public static double carouselPosAng = Math.toRadians(270);

    public static double parkX = -60;
    public static double parkY = 42;
    public static double parkAng = Math.toRadians(180);

    public static String goal = "";

    public static Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
    public static Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
    public static Pose2d carouselPosR = new Pose2d(carouselPosx, -carouselPosy, carouselPosAng);
    public static Pose2d parkR = new Pose2d(parkX, -parkY, parkAng);
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        Pose2d startPosB = new Pose2d(startx, starty, startAng);
        Pose2d startPosR = new Pose2d(startx, -starty, -startAng);

        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                .setDimensions(11.838583, 14.4882 )
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->
            
                        drive.trajectorySequenceBuilder(startPosR)
                        .waitSeconds(1)
                        .setReversed(true)
                        .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.releaseHard();
                        })
                        .waitSeconds(1)
                        // slides
                        .lineToSplineHeading(carouselPosR)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //carousel.run(false, true);
                        })
                        .waitSeconds(7)
                        // carousel
                        .lineToSplineHeading(parkR)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //carousel.run(false, false);
                        })
                        .lineTo()
                        .build()

                );


        


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotBlue)
                //.addEntity(myBotRed)
                .start();
    }
}