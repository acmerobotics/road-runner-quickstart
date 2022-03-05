package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class MeepMeepTestingParkSide {

    public static double startx = 15.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = 7;
    public static double scoreHubPosy = 43;

    public static double scoreHubPosAngB = 40;
    public static double repositionX = 15.0;
    public static double reposistionY = 71.5;

    public static double distanceForwards = 30;
    public static double strafeDistance = 24;

    public static String goal = "";

    public static Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
    public static Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
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
        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 9.85)
                .setDimensions(11.838583, 14.4882 )
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->
            
                        drive.trajectorySequenceBuilder(startPos)
                        .waitSeconds(2)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(scoreHubPosB, Math.toRadians(scoreHubPosAngB)))
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                            // scoringMech.release();
                        })
                        .waitSeconds(1)
                        // .lineToLinearHeading(repositionB)
                        .splineTo(new Vector2d(repositionX, reposistionY - 4), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY), Math.toRadians(
                                0))
                        //.lineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY,Math.toRadians(0)))
                        .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                            // intake here
                        })
                        .setReversed(true)
                        .waitSeconds(1)

                        .splineTo(new Vector2d(repositionX + 5, reposistionY), Math.toRadians(180))

                        .splineTo(new Vector2d(scoreHubPosx, scoreHubPosy),
                                Math.toRadians(220))
                        .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                            //score here
                        })
                        .waitSeconds(1)
                        .setReversed(false)
                        .splineTo(new Vector2d(repositionX, reposistionY - 4), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY), Math.toRadians(
                                0))
                        //.lineToLinearHeading(new Pose2d(repositionX + distanceForwards, reposistionY,Math.toRadians(0)))
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