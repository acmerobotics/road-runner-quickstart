package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class cycles {

    public static double startx = 15.0;
    public static double starty = -70.0;
    public static double startAng = Math.toRadians(270);

    public static double scoreHubPosx = 0;
    public static double scoreHubPosy = -52;

    public static double scoreHubPosAngB = 360-65;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double preSplineY = 53.5;
    public static double bEnterX = 30;
    public static double bEnterY = -71.5;
    public static double warehouseX = 51;
    public static double bExitX = 30;
    public static double inc = 0;

    public static double distanceForwards = 30;
    public static double strafeDistance = 24;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        Pose2d startPosB = new Pose2d(startx, starty, startAng);
        Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
        Pose2d startPos = new Pose2d(startx, starty, startAng);

        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Vector2d bEnter = new Vector2d(bEnterX, bEnterY);

        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 9.85)
                .setDimensions(11.838583, 14.4882 )
                //.setStartPose(startPos)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPosB)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            scoringMech.releaseHard();
//                            intake.intake(1);
                        })
                        //.waitSeconds(.1)
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bExitX, bEnterY, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        //.waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            scoringMech.toggle("highgoal");
                        })
                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                            intake.outake(1.0);
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            scoringMech.releaseHard();
//                            intake.intake(1);
                        })
                        //.waitSeconds(.1)
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX+1, bEnterY))
                        //.waitSeconds(0.1)
//                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            scoringMech.toggle("highgoal");
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                            intake.outake(1.0);
//                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            readjustLocale(drive);
//                            scoringMech.releaseHard();
//                            intake.intake(1);
//                        })
//                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX+3, bEnterY))
                        //.waitSeconds(0.1)
//                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            scoringMech.toggle("highgoal");
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                            intake.outake(1.0);
//                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy+1), Math.toRadians(scoreHubPosAngB+180))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            scoringMech.releaseHard();
//                            intake.intake(1);
//                        })
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX+5, bEnterY))
                        //.waitSeconds(0.1)
//                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                            scoringMech.toggle("highgoal");
//                        })
//                        .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                            intake.outake(1.0);
//                        })

                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy+1), Math.toRadians(scoreHubPosAngB+180))
//                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                            scoringMech.releaseHard();
//                            intake.intake(1);
//                        })
//                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX+6, bEnterY))
                                .build()
                );

        RoadRunnerBotEntity myBotRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 11)
                // .setStartPose(startPos)
                .setDimensions(11.839, 14.141)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPosR)
                        .setReversed(true)
                        .lineTo(new Vector2d(15, -60))
                        .splineTo(new Vector2d(20, -55), Math.toRadians(0))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .splineTo(new Vector2d(15, -60), Math.toRadians(270))
                        .lineTo(new Vector2d(15, -70))
                        .setReversed(true)
                        .lineTo(new Vector2d(15, -60))
                        .splineTo(new Vector2d(20, -55), Math.toRadians(0))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .splineTo(new Vector2d(15, -60), Math.toRadians(270))
                        .lineTo(new Vector2d(startx, -starty))
                        .setReversed(true)
                        .lineTo(new Vector2d(15, -60))
                        .splineTo(new Vector2d(20, -55), Math.toRadians(0))
                        .waitSeconds(0.1)
                        .setReversed(false)
                        .splineTo(new Vector2d(15, -60), Math.toRadians(270))
                        .lineTo(new Vector2d(startx, -starty))
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