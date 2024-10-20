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
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(56,48), Math.toRadians(245))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(33,9), Math.toRadians(180))
                .waitSeconds(1.2)
                .build());

        BlueHumanSide.runAction(BlueHumanSide.getDrive().actionBuilder(new Pose2d(-15, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-11,48))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(-30,48), Math.toRadians(255))
                .strafeToLinearHeading(new Vector2d(-38,14), Math.toRadians(260))
                .strafeToLinearHeading(new Vector2d(-45,12), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-45,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-45,13), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,13), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,13), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-61,13), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-61,50), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-48,54), Math.toRadians(270))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-4,45), Math.toRadians(270))
                .waitSeconds(1.86)
                .strafeTo(new Vector2d(-48,54))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-4,45))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(-42,50), Math.toRadians(270))
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
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(-56,-48), Math.toRadians(65))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-33,-9), Math.toRadians(0))
                .waitSeconds(1.2)
                .build());

        RedHumanSide.runAction(RedHumanSide.getDrive().actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))
                .strafeTo(new Vector2d(11,-48))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(30,-48), Math.toRadians(75))
                .strafeToLinearHeading(new Vector2d(38,-14), Math.toRadians(80))
                .strafeToLinearHeading(new Vector2d(45,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45,-50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(45,-13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(56,-50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(56,-13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(61,-13), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(61,-50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(48,-54), Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(4,-45), Math.toRadians(90))
                .waitSeconds(1.86)
                .strafeTo(new Vector2d(48,-54))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(4,-45))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(42,-50), Math.toRadians(90))
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