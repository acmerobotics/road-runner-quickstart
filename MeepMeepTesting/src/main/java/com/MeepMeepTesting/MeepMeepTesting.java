package com.MeepMeepTesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, 65, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(13, 45, Math.toRadians(-120)))
                                .lineToLinearHeading(new Pose2d(20, 50, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(46, 36))
                                .lineToLinearHeading(new Pose2d(23, 11, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-53, 11))
                                .lineToConstantHeading(new Vector2d(23, 11))
                                .lineToLinearHeading(new Pose2d(46, 36, Math.toRadians(0)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}