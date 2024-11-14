
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Curves;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepShreyanshTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(new Vector2d(48, -50), Math.toRadians(-90)))
        //new TrajectoryBuilder(new Pose2d())
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(new Vector2d(48, -10), Math.toRadians(-90)), Math.toRadians(-90))

                        .setReversed(true)
                        //.splineToConstantHeading(new Vector2d(52,-45), Math.toRadians(90)
                        .splineToConstantHeading(new Vector2d(56,-10), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(56,-53), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(0)), Math.toRadians(0))
                        .strafeToConstantHeading(new Vector2d(60,-53))
                .build());

        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .start();
    }
}



