package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class moremeeptesting {

    public static double startx = -36.0;
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

    public static double reposX = -34;
    public static double reposY = 36;

    public static double preSweepY = 48;
    public static double sweepX = -40;
    public static double sweepY = 67;

    public static double duckX = -58;
    public static double duckY = 65;

    public static String goal = "midgoal";


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d carouselPosB = new Pose2d(carouselPosx, carouselPosy, carouselPosAng);
        Pose2d parkB = new Pose2d(parkX, parkY, parkAng);
        Pose2d startPosB = new Pose2d(startx, starty, startAng);
        Pose2d reposition = new Pose2d(reposX, reposY, Math.toRadians(90));
        Vector2d preSweep = new Vector2d(reposX, preSweepY);
        Vector2d sweepPos = new Vector2d(sweepX, sweepY);
        Pose2d postSweep = new Pose2d(duckX, sweepY, Math.toRadians(90));
        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);

        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                .setDimensions(11.838583, 14.4882 )
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(startPosB)
                                .waitSeconds(1)
                                .setReversed(true)
                                .splineTo(scoreHubPosB,Math.toRadians(scoreHubPosAngB))
                                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                                   // scoringMech.release();
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
                                   // carousel.run(false,false);
                                })
                                .lineToSplineHeading(reposition)
                                .lineTo(preSweep)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.intake(1);
                                })
                                .splineTo(sweepPos, Math.toRadians(180))
                                //.splineTo(new Vector2d(duckX, duckY), Math.toRadians(180))
                                .lineToLinearHeading(postSweep)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                   // intake.intake(0);
                                })
                                .setReversed(true)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scoringMech.toggle("highgoal");
                                })
                                .splineTo(scoreHubPosB, Math.toRadians(scoreHubPosAngB))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //.release();
                                })
                                .waitSeconds(1)
                                .lineToSplineHeading(parkB)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //c//arousel.run(false, false);
                                })
                                .build()

                );

        /*RoadRunnerBotEntity myBotRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                // .setStartPose(startPos)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPosR)
                        .waitSeconds(2)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(scoreHubPosR, Math.toRadians(scoreHubPosAngR)))                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                            //scoringMech.release();
                        })
                        .waitSeconds(1)
                        //.lineToLinearHeading(repositionB)
                        .lineTo(new Vector2d(scoreHubPosx, -reposistionY +20))
                        .splineToSplineHeading(new Pose2d(repositionX+5, -reposistionY, Math.toRadians(0)), Math.toRadians(0))
                        .lineTo(new Vector2d(repositionX + distanceForwards, -reposistionY))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(repositionX+5, -reposistionY), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(scoreHubPosx, -scoreHubPosy, Math.toRadians(-40)), Math.toRadians(90))
                        .waitSeconds(1)
                        .lineTo(new Vector2d(scoreHubPosx, -reposistionY +20))
                        .splineToSplineHeading(new Pose2d(repositionX+5, -reposistionY, Math.toRadians(0)), Math.toRadians(0))
                        .lineTo(new Vector2d(repositionX + distanceForwards, -reposistionY))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(repositionX+5, -reposistionY), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(scoreHubPosx, -scoreHubPosy, Math.toRadians(-40)), Math.toRadians(90))
                        .waitSeconds(1)
                        .lineTo(new Vector2d(scoreHubPosx, -reposistionY +20))
                        .splineToSplineHeading(new Pose2d(repositionX+5, -reposistionY, Math.toRadians(0)), Math.toRadians(0))
                        .lineTo(new Vector2d(repositionX + distanceForwards, -reposistionY))
                        .strafeLeft(strafeDistance)
                        .build()

                );*/





        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotBlue)
                //.addEntity(myBotRed)
                .start();
    }
}