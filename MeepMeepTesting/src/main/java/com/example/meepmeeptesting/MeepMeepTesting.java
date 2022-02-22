package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class MeepMeepTesting {

    public static double startx = -35.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 43;

    public static double scoreHubPosAngB = -25;
    public static double scoreHubPosAngR = 25;

    public static double carouselPosx = -62;
    public static double carouselPosy = 62;
    public static double carouselPosAng = Math.toRadians(180);

    public static double parkX = -60;
    public static double parkY = 40;
    public static double parkAng = Math.toRadians(180);

    public static double tempX = -34;
    public static double tempY = -40;

    public static double reposX = -34;
    public static double reposY = 36;

    public static double duckX = -58;
    public static double duckY = -68;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        Pose2d startPosB = new Pose2d(startx, starty, startAng);
        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Pose2d carouselPosB = new Pose2d(carouselPosx, carouselPosy, carouselPosAng);
        Pose2d parkB = new Pose2d(parkX, parkY, parkAng);

        Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
        Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
        Pose2d carouselPosR = new Pose2d(carouselPosx, -carouselPosy, carouselPosAng);
        Pose2d parkR = new Pose2d(parkX, -parkY, parkAng);

        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                //.setDimensions()
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPosB)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(scoreHubPosB,Math.toRadians(scoreHubPosAngB))
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                  //  scoringMech.release();
                                })
                                .waitSeconds(1)
                                //slides
                                .lineToSplineHeading(carouselPosB)
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                    //carousel.run(true,false);
                                })
                                .waitSeconds(4)
                                //carousel
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                    //carousel.run(false,false);
                                })
                                .lineToSplineHeading(new Pose2d(reposX, reposY, Math.toRadians(90)))
                                .lineTo(new Vector2d( reposX, reposY + 12))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.intake(1);
                                })
                                .splineTo(new Vector2d(-40, -duckY+2), Math.toRadians(180))
                                //.splineTo(new Vector2d(duckX, duckY), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(duckX, -duckY+2, Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.intake(0);
                                })
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scoringMech.toggle("highgoal");
                                })
                                .splineTo(scoreHubPosB, Math.toRadians(scoreHubPosAngB))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(scoreHubPosx, scoreHubPosy+10, Math.toRadians(0)))
                                .forward(30)
                                .splineToSplineHeading(new Pose2d(15, 71.5, Math.toRadians(0)), Math.toRadians(0))
                                .forward(30)
                                .build()

                );

        RoadRunnerBotEntity myBotRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                // .setStartPose(startPos)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPosR)
                        .setReversed(true)
                        .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.release();
                        })
                        //.waitSeconds(1)
                        // slides
                        .lineToSplineHeading(carouselPosR)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // carousel.run(true,false);
                        })
                        //.waitSeconds(7)
                        // carousel
                        /*.lineToSplineHeading(parkR)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // carousel.run(false,false);
                        })*/
                        .lineToSplineHeading(new Pose2d(tempX, tempY, Math.toRadians(270)))
                        .lineTo(new Vector2d( tempX, tempY-5))
                        .splineTo(new Vector2d(-40, duckY-2), Math.toRadians(180))
                        //.splineTo(new Vector2d(duckX, duckY), Math.toRadians(180))
                        .lineToLinearHeading(new Pose2d(duckX, duckY-2, Math.toRadians(225)))
                        .setReversed(true)
                        .splineTo(scoreHubPosR, Math.toRadians(scoreHubPosAngR))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(scoreHubPosx, -scoreHubPosy-10, Math.toRadians(0)))
                        .forward(30)
                        .splineToSplineHeading(new Pose2d(15, -71.5, Math.toRadians(0)), Math.toRadians(0))
                        .build()

                );


        


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotBlue)
                .addEntity(myBotRed)
                .start();
    }
}