package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-8, -61, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-57, -50, Math.toRadians(224)), Math.toRadians(180))
                        .waitSeconds(3)
                        .splineToLinearHeading(new Pose2d(-48, -47, Math.toRadians(224)), Math.toRadians(0))
                        //.splineToConstantHeading(new Vector2d(-50, -45), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-47, -40, Math.toRadians(90)), Math.toRadians(100))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-56, -52, Math.toRadians(226)), Math.toRadians(180))
                        .waitSeconds(3)
                        .splineToConstantHeading(new Vector2d(-50, -45), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(90)), Math.toRadians(200))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(-56, -52, Math.toRadians(226)), Math.toRadians(180))
                        .waitSeconds(3)
                        .splineToConstantHeading(new Vector2d(-48, -48), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-30, 12, Math.toRadians(0)), Math.toRadians(70))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}