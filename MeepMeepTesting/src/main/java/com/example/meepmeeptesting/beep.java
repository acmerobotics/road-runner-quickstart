package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class beep {

    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = 6;
    public static double scoreHubPosy = 52;

    public static double scoreHubPosAngB = 60;
    public static double scoreHubPosAngR = -40;

    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double preSplineY = 52.01;
    public static double bEnterX = 20;
    public static double bExitX = 30;
    public static double bEnterY = 68;
    public static double warehouseX = 48;
    public static double bExitY = -70.5;
    public static double inc = 0;
    public static Pose2d startPos = new Pose2d(startx, starty, startAng);

    Pose2d startPosB = new Pose2d(startx, starty, startAng);
    Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
    Pose2d repositionB = new Pose2d(repositionX, reposistionY, Math.toRadians(0));
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        Pose2d startPosB = new Pose2d(startx, starty, startAng);
        Pose2d startPosR = new Pose2d(startx, -starty, -startAng);

        Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);

        Pose2d repositionB = new Pose2d(repositionX, reposistionY, Math.toRadians(0));
        Vector2d preSpline = new Vector2d(scoreHubPosx, preSplineY);
        Vector2d bEnter = new Vector2d(bExitX, bEnterY);
        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 9.85)
                .setDimensions(11.838583, 14.4882 )
                //.setStartPose(startPos)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPos)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        .waitSeconds(.1)
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bExitX, bEnterY, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        .waitSeconds(.1)
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        //.lineTo(preSpline)
                        .setReversed(false)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-1, bEnterY))
                        .waitSeconds(0.1)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            //scoringMech.toggle("highgoal");
                            // drive.acquirerRuns = false;
                        })
                        .setReversed(true)
                        .lineTo(new Vector2d(bExitX, bEnterY))
                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy), Math.toRadians(scoreHubPosAngB+180))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.releaseHard();
                            // drive.acquirerRuns = true;
                        })
                        .setReversed(false)
                        //.lineTo(preSpline)
                        .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(warehouseX-10, bEnterY))
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