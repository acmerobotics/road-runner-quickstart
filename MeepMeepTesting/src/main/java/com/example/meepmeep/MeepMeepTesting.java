package com.example.meepmeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

import org.graalvm.compiler.lir.LIRInstruction;

public class MeepMeepTesting {
    public static void main(String args[]) {
        MeepMeep nm = new MeepMeep(800);

        // declaring start pos
        Pose2d startPose = new Pose2d(-36,-36,Math.toRadians(0));

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(nm)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(83, 30, Math.toRadians(60), Math.toRadians(60), 15)
                .setDimensions(16,16)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(-36,0), Math.toRadians(90))
                                .splineTo(new Vector2d(0,56), Math.toRadians(180))
                                .splineTo(new Vector2d(-56,0), Math.toRadians(270))
                                .splineTo(new Vector2d(0,-56), Math.toRadians(0))
                                .build()
                );


        nm.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot1)
                .start();
    }
}
