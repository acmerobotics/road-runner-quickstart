package com.example.meepmep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity BlueBasketSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity BlueHumanSide = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity RedBasketSide = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        RoadRunnerBotEntity RedHumanSide = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        BlueBasketSide.runAction(BlueBasketSide.getDrive().actionBuilder(new Pose2d(15, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(9,48))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(56,48), Math.toRadians(245))
                .waitSeconds(3)
                .turn(Math.toRadians(30))
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(55))
                .waitSeconds(1)
                .turn(Math.toRadians(-55))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(33,9), Math.toRadians(180))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(56,48), Math.toRadians(245))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(33,9), Math.toRadians(180))
                .waitSeconds(2)
                .build());

        BlueHumanSide.runAction(BlueHumanSide.getDrive().actionBuilder(new Pose2d(-15, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-11,48))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-56,48), Math.toRadians(295))
                .waitSeconds(3)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .turn(Math.toRadians(25))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-4,52), Math.toRadians(270))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56,48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-4,52))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56,48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-4,52))
                .waitSeconds(2)
                .build());


        RedBasketSide.runAction(RedBasketSide.getDrive().actionBuilder(new Pose2d(-15, -63, Math.toRadians(90)))
                .strafeTo(new Vector2d(-9,-48))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-56,-48), Math.toRadians(65))
                .waitSeconds(3)
                .turn(Math.toRadians(30))
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(55))
                .waitSeconds(1)
                .turn(Math.toRadians(-55))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,-9), Math.toRadians(0))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(-56,-48), Math.toRadians(65))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,-9), Math.toRadians(0))
                .waitSeconds(2)
                .build());

        RedHumanSide.runAction(RedHumanSide.getDrive().actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))
                .strafeTo(new Vector2d(11,-48))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(56,-48), Math.toRadians(115))
                .waitSeconds(3)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(-25))
                .waitSeconds(1)
                .turn(Math.toRadians(25))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(4,-52), Math.toRadians(90))
                .waitSeconds(2)
                .strafeTo(new Vector2d(56,-48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(4,-52))
                .waitSeconds(2)
                .strafeTo(new Vector2d(56,-48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(4,-52))
                .waitSeconds(2)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueBasketSide)
                .addEntity(BlueHumanSide)
                .addEntity(RedBasketSide)
                .addEntity(RedHumanSide)
                .start();
    }
}