package com.example.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

public class MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep nm = new MeepMeep(1200);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(nm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(83, 30, Math.toRadians(60), Math.toRadians(60), 16.88)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 90))
                                .splineToConstantHeading(new Vector2d(40,40), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(90))
                                .build()
                );

        nm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
