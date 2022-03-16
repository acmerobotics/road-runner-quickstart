package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.MathKt;

public class MeepMeepTesting {


    public static double scoreHubPosx = 0;
    public static double scoreHubPosy = 38;

    public static double wareHousePosX = 48;
    public static double warhousePosY = 64;

    public static Pose2d origin = new Pose2d(15,70,Math.toRadians(90));
    public static Pose2d startPos = new Pose2d(scoreHubPosx,scoreHubPosy,Math.toRadians(45));
    public static Pose2d endPos = new Pose2d(wareHousePosX,warhousePosY, Math.toRadians(0));
    public static Pose2d midWayPos = new Pose2d(15,64,Math.toRadians(0));
    public static Vector2d endPosVector = new Vector2d(wareHousePosX,warhousePosY);
    public static Vector2d startPosVector = new Vector2d(scoreHubPosx,scoreHubPosy);

    public static Pose2d firstRepos = new Pose2d(scoreHubPosx + 8 , scoreHubPosy + 8, Math.toRadians(22.5));
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBotBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, Math.toRadians(180), Math.toRadians(180), 9.85)
                //.setDimensions()
                //.setStartPose(startPos)
                .followTrajectorySequence(drive ->

                        drive.trajectorySequenceBuilder(origin)
                                //.lineToLinearHeading(startPos)
                                .setReversed(true)
                                .splineTo(pose2Vector(firstRepos), Math.toRadians(180) + firstRepos.getHeading())
                                .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                                // .setReversed(true)
                                // .splineTo(pose2Vector(startPos), startPos.getHeading())
                                .setReversed(false)
                                .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                                .splineTo(pose2Vector(endPos), endPos.getHeading())
                                .setReversed(true)
                                
                                .splineToLinearHeading(midWayPos, midWayPos.getHeading() + Math.toRadians(180))
                                .splineTo(pose2Vector(startPos),  startPos.getHeading() + Math.toRadians(180))
                                .setReversed(false)

                                //cycle1
                        //         .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                        //         .splineTo(pose2Vector(endPos), endPos.getHeading())
                        //         .setReversed(true)

                        //         .splineToLinearHeading(midWayPos, midWayPos.getHeading() + Math.toRadians(180))
                        //         .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                        //         .setReversed(false)

                        //         //cycle 2
                        //         .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                        //         .splineTo(pose2Vector(endPos), endPos.getHeading())
                        //         .setReversed(true)

                        //         .splineToLinearHeading(midWayPos, midWayPos.getHeading() + Math.toRadians(180))
                        //         .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                        //         .setReversed(false)

                        //         //cycle 3
                        //         .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                        //         .splineTo(pose2Vector(endPos), endPos.getHeading())
                        //         .setReversed(true)

                        //         .splineToLinearHeading(midWayPos, midWayPos.getHeading() + Math.toRadians(180))
                        //         .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                        //         .setReversed(false)

                        //         //cycle 4
                        //         .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                        //         .splineTo(pose2Vector(endPos), endPos.getHeading())
                        //         .setReversed(true)

                                
                        //         .splineToLinearHeading(midWayPos, midWayPos.getHeading() + Math.toRadians(180))
                        //         .splineTo(pose2Vector(startPos), startPos.getHeading() + Math.toRadians(180))
                        //         .setReversed(false)

                        //         //cycle 5
                        //        .splineTo(pose2Vector(midWayPos), midWayPos.getHeading())
                        //         .splineTo(pose2Vector(endPos), endPos.getHeading())
                        //         .setReversed(true)


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