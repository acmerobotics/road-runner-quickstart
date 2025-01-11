package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 65, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(8, -62, Math.toRadians(90)))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(8, -36, Math.toRadians(90)), Math.toRadians(90))
                        .addTemporalMarker(2, () -> {

                })
                        .waitSeconds(3)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(44, -12, Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(44, -53, Math.toRadians(90)), Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(52, -12, Math.toRadians(90)), Math.toRadians(0))
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(52, -53, Math.toRadians(90)) , Math.toRadians(270))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(42, -53, Math.toRadians(90)), Math.toRadians(180))
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(0, -34, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(42, -53, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(2, -34, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(42, -53, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(4, -34, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(42, -53, Math.toRadians(90)), Math.toRadians(0))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}