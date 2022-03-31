package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class CurvyStraight {


        public static double startx = 15.0;
        public static double starty = 70.0;
        public static double startAng = Math.toRadians(90);

        public static double scoreHubPosx = 6;
        public static double scoreHubPosy = 46;
        public static double scoreHubXOffset = -8.0;
        public static double scoreHubYOffset = 5;
        public static double scoreHubAngleOffset = 90;

        public static double scoreHubPosAngB = 60;
        public static double scoreHubPosAngR = -40;

        public static double repositionX = scoreHubPosx;
        public static double reposistionY = 71.5;

        public static double preSplineY = 60;
        public static double preSplineX = 2;

        public static double preSplineVersY = 60;
        public static double preSplineVersX = 5;

        public static double bEnterX = 20;

        //public static double bEnterY = 73;
        public static double bEnterY = 65;

        public static double warehouseX = 43;
        public static double bExitY = 73;
        public static double inc = 0;
        public static Pose2d startPos = new Pose2d(startx, starty, startAng);
        public static double xmod = 7;
        public static double ymod = 5;
        public static double localeReadjustX = 0;
        public static double wallYEstimate = 70 + 2.38;
        public static double localeReadjustY = 0;

        public static double depoIncrement = 1;
        public static double intakeAngle = -20;
        public static String goal = "highgoal";

        public static Pose2d startPosB = new Pose2d(startx, starty, startAng);
        public static Pose2d startPosR = new Pose2d(startx, -starty, -startAng);

        public static Vector2d scoreHubPosB = new Vector2d(scoreHubPosx, scoreHubPosy);
        public static Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);

        public static Pose2d repositionB = new Pose2d(repositionX, reposistionY, Math.toRadians(0));
        public static Vector2d preSpline = new Vector2d(preSplineX, preSplineY);
        public static Vector2d preSplineVers = new Vector2d(preSplineVersX, preSplineVersY);
        public static double preSplineVersAng = 30;

        public static Vector2d bEnterLine = new Vector2d(bEnterX-10, bEnterY - 10);

        public static Vector2d bEnter = new Vector2d(bEnterX, bEnterY);

        public static Vector2d bEnter2 = new Vector2d(bEnterX, bEnterY - inc);
        public static Vector2d bEnter3 = new Vector2d(bEnterX, bEnterY - 2 * inc);
        public static Vector2d bEnter4 = new Vector2d(bEnterX, bEnterY - 3 * inc);
        public static Vector2d bEnter5 = new Vector2d(bEnterX, bEnterY - 4 * inc);
        public static Vector2d bExit = new Vector2d(bEnterX, bExitY);
        public static Vector2d bExit2 = new Vector2d(bEnterX, bEnterY - 2 * inc);
        public static Vector2d bExit3 = new Vector2d(bEnterX, bEnterY - 4 * inc);
        public static Vector2d bExit4 = new Vector2d(bEnterX, bEnterY - 6 * inc);
        public static Vector2d wareHouse = new Vector2d(warehouseX, bEnterY);
        public static Vector2d wareHouse2 = new Vector2d(warehouseX, bEnterY - inc);
        public static Vector2d wareHouse3 = new Vector2d(warehouseX, bEnterY - 2 * inc);
        public static Vector2d wareHouse4 = new Vector2d(warehouseX, bEnterY - 3 * inc);
        public static Vector2d wareHouse5 = new Vector2d(warehouseX, bEnterY - 4 * inc);
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 9.85)
                //.setDimensions()
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(
                                        startPos)
                                .setReversed(true)
                .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true;                     //This section is "go to score hub from startpos, score and start intake"
                })


                .lineTo(bEnterLine)
                .splineTo(bEnter, Math.toRadians(0))
                .lineTo(new Vector2d(warehouseX, bEnterY))
                //.setReversed(false)
                .lineTo(bEnter)
                .splineTo(bEnterLine, Math.toRadians(scoreHubPosAngB + 180))
                .lineTo(scoreHubPosB)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // scoringMech.releaseHard();
                    // drive.acquirerRuns = true; //This section is "go to score hub from startpos,
                    // score and start intake"
                })



                .waitSeconds(.1)

                //-----------------------------------------------------------------------------------END OF DUMPING PRELOAD
                //-----------------------------------------------------------------------------------BEGINNING OF CYCLE 1
                // .lineTo(preSpline)
                // .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                // .UNSTABLE_addTemporalMarkerOffset(0,()->{

                // })
                // .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                // .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                //     // drive.acquirerReverse = true;
                //     // scoringMech.toggle("highgoal");
                //     // drive.acquirerRuns = false;              //This section is "spline into the barrier, then into warehouse, then stop intaking after .1 seconds
                // })
                //     //-----------------------------------------------------------------------------------
                // .lineTo(new Vector2d(bEnterX-5, bExitY))
                // .splineTo(new Vector2d(scoreHubPosx + scoreHubXOffset, scoreHubPosy + scoreHubYOffset), Math.toRadians(scoreHubAngleOffset+180))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     // scoringMech.releaseHard();
                //     // drive.acquirerReverse = false;   //This section is "line out of the barrier, then spline to the score hub"
                // })
                // //-----------------------------------------------------------------------------------END OF CYCLE 1
                // //-----------------------------------------------------------------------------------BEGINNING OF CYCLE 2
                // .setReversed(false)

                // .splineTo(preSplineVers, Math.toRadians(preSplineVersAng))
                // .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     //readjustLocale(drive);

                // })
                // .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))

                // //.splineTo(new Vector2d(warehouseX+depoIncrement * 2, bEnterY-ymod), Math.toRadians(intakeAngle))

                // .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                //     // scoringMech.toggle("highgoal");
                //     // drive.acquirerReverse = true;
                // })
                // .waitSeconds(0.1)
                // .setReversed(true)
                // .splineTo(new Vector2d(bEnterX+xmod, bEnterY), Math.toRadians(180))
                // .splineTo(new Vector2d(scoreHubPosx + scoreHubXOffset, scoreHubPosy), Math.toRadians(scoreHubAngleOffset+180))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     // scoringMech.releaseHard();
                //     // drive.acquirerReverse = false;
                // })
                // //-----------------------------------------------------------------------------------END OF CYCLE 2
                // //-----------------------------------------------------------------------------------BEGINNING OF CYCLE 3
                // //.lineTo(preSpline)
                // .setReversed(false)

                // .splineTo(preSplineVers, Math.toRadians(preSplineVersAng))

                // .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     //readjustLocale(drive);

                // })
                // .lineToLinearHeading(new Pose2d(warehouseX+depoIncrement * 2, bEnterY))

                // .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                //     // scoringMech.toggle("highgoal");
                //     // drive.acquirerReverse = true;
                // })
                // .setReversed(true)
                // .lineTo(new Vector2d(bEnterX, bExitY))
                // .splineTo(new Vector2d(scoreHubPosx + scoreHubXOffset, scoreHubPosy + scoreHubYOffset), Math.toRadians(scoreHubAngleOffset+180))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     // scoringMech.releaseHard();
                //     // drive.acquirerReverse = false;
                // })
                // //-----------------------------------------------------------------------------------END OF CYCLE 3
                // //-----------------------------------------------------------------------------------BEGINNING OF CYCLE 4
                // //.lineTo(preSpline)
                // .setReversed(false)
                // .splineTo(preSplineVers, Math.toRadians(preSplineVersAng))

                // .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     //readjustLocale(drive);

                // })
                // .splineTo(new Vector2d(warehouseX+depoIncrement * 3, bEnterY-ymod), Math.toRadians(intakeAngle))

                // .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                //     // scoringMech.toggle("highgoal");
                //     // drive.acquirerReverse = true;
                // })
                // .waitSeconds(0.1)
                // .setReversed(true)
                // .splineTo(new Vector2d(bEnterX+xmod, bEnterY), Math.toRadians(180))
                // .splineTo(new Vector2d(scoreHubPosx + scoreHubXOffset, scoreHubPosy + scoreHubYOffset), Math.toRadians(scoreHubAngleOffset+180))

                // .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                //     // scoringMech.releaseHard();
                //     // drive.acquirerReverse = false;
                // })
                // //.lineTo(preSpline)
                // .setReversed(false)
                // .splineTo(preSplineVers, Math.toRadians(preSplineVersAng))

                // .splineToSplineHeading(new Pose2d(bEnter, Math.toRadians(0)), Math.toRadians(0))
                // .lineToLinearHeading(new Pose2d(warehouseX, bEnterY))
                .build()

        );


                // RoadRunnerBotEntity myBotRed = new DefaultBotBuilder(meepMeep)
                // // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                // .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                // // .setDimensions()
                // // .setStartPose(startPos)
                // .followTrajectorySequence(drive ->

                // drive.trajectorySequenceBuilder(startPos)
                        
                //         .build()

                // );


        


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotBlue)
                .start();
    }

    public static Vector2d pose2Vector(Pose2d givenPose){
        return new Vector2d(givenPose.getX(),givenPose.getY());
    }
}