package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8, -61, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(8, -34, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(8, -42, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48, -38, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(50, -58, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(57, -38, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(50, -58, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(50, -48, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(2, -34, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(8, -46, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(50, -58, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(2)
                        .splineToLinearHeading(new Pose2d(50, -48, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-7, -34, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(7, -48, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(40, -48, Math.toRadians(90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(28, 10, Math.toRadians(90)), Math.toRadians(100))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}