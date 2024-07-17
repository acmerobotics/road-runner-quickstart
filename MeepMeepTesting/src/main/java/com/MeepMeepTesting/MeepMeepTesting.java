package com.MeepMeepTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 65, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(14,64, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(13, 45, Math.toRadians(-120)))//pune pixel pe right


                                .lineToLinearHeading(new Pose2d(20, 45, Math.toRadians(0))) //rotire
                                .lineToConstantHeading(new Vector2d(46, 30)) //merge la backdrop



                                .lineToLinearHeading(new Pose2d(23, 11, Math.toRadians(180))) //rotire+diag

                                .lineToLinearHeading(new Pose2d(-53, 11, Math.toRadians(180)) //trece pe sub door si aj la stack



                                ) .build()
                                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}