package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.graalvm.compiler.lir.LIRInstruction;

public class MeepMeepTesting {
    public static double tuningNumber = 36;
    public static double tuningTimer = 1;


    public static double startx = 0;
    public static double starty = -72;

    public static double bankcurveX = -5;
    public static double bankcurveY = starty + 22;
    public static int cycles = 4;

    public static double maxVel = 40;

    public static double fakex = 0;
    public static double fakey = 72;

    public static double fakecurvex = -3.5;
    public static double fakecurvey = fakey - 22;

    public static double duckx = 5;
    public static double ducky = -83;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        Pose2d startPos = new Pose2d(startx,starty, Math.toRadians(180));
        Pose2d duckPos = new Pose2d(startx-tuningNumber, starty, Math.toRadians(90));
        Pose2d fakePos = new Pose2d(fakex, fakey, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.85)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(duckPos)
                                /*.setReversed(false)
                                .back(18)
                                .waitSeconds(0.1)
                                .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(bankcurveX,bankcurveY+tuningNumber,Math.toRadians(90)))
                                .waitSeconds(0.1)
                                //start of allahhuackbar
                                .lineToSplineHeading(new Pose2d(bankcurveX,bankcurveY,Math.toRadians(90)))


                                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineTo(new Vector2d(bankcurveX-2,bankcurveY),Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(bankcurveX-2,bankcurveY+tuningNumber,Math.toRadians(90)))
                                .waitSeconds(0.1)
                                //start of allahhuackbar
                                .lineToSplineHeading(new Pose2d(bankcurveX-2,bankcurveY,Math.toRadians(90)))


                                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineTo(new Vector2d(bankcurveX-4,bankcurveY),Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(bankcurveX-4,bankcurveY+tuningNumber,Math.toRadians(90)))
                                .waitSeconds(0.1)
                                //start of allahhuackbar
                                .lineToSplineHeading(new Pose2d(bankcurveX-4,bankcurveY,Math.toRadians(90)))

                                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .splineTo(new Vector2d(bankcurveX-6,bankcurveY),Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(bankcurveX-6,bankcurveY+tuningNumber,Math.toRadians(90)))
                                .waitSeconds(0.1)
                                //start of allahhuackbar
                                .lineToSplineHeading(new Pose2d(bankcurveX-6,bankcurveY,Math.toRadians(90)))

                                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
                                .waitSeconds(0.1)
                                .setReversed(false)
                                .build()*/
                                .lineToSplineHeading(new Pose2d(duckx, ducky, Math.toRadians(0)))

                                .build()

                );

        RoadRunnerBotEntity otherside = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.85)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(fakePos)
                                        .setReversed(false)
                                        .back(18)
                                        .waitSeconds(0.1)
                                        .splineTo(new Vector2d(fakecurvex,fakecurvey),Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(fakecurvex,fakecurvey-tuningNumber,Math.toRadians(270)))
                                        .waitSeconds(0.1)
                                        //start of allahhuackbar
                                        .lineToSplineHeading(new Pose2d(fakecurvex,fakecurvey,Math.toRadians(270)))


                                        .splineTo(new Vector2d(18, fakey), Math.toRadians(0))
                                        .waitSeconds(0.1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(fakecurvex-2,fakecurvey),Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(fakecurvex-2,fakecurvey-tuningNumber,Math.toRadians(270)))
                                        .waitSeconds(0.1)
                                        //start of allahhuackbar
                                        .lineToSplineHeading(new Pose2d(fakecurvex-2,fakecurvey,Math.toRadians(270)))


                                        .splineTo(new Vector2d(18, fakey), Math.toRadians(0))
                                        .waitSeconds(0.1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(fakecurvex-4,fakecurvey),Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(fakecurvex-4,fakecurvey-tuningNumber,Math.toRadians(270)))
                                        .waitSeconds(0.1)
                                        //start of allahhuackbar
                                        .lineToSplineHeading(new Pose2d(fakecurvex-4,fakecurvey,Math.toRadians(270)))

                                        .splineTo(new Vector2d(18, fakey), Math.toRadians(0))
                                        .waitSeconds(0.1)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(fakecurvex-6,fakecurvey),Math.toRadians(270))
                                        .lineToSplineHeading(new Pose2d(fakecurvex-6,fakecurvey-tuningNumber,Math.toRadians(270)))
                                        .waitSeconds(0.1)
                                        //start of allahhuackbar
                                        .lineToSplineHeading(new Pose2d(fakecurvex-6,fakecurvey,Math.toRadians(270)))

                                        .splineTo(new Vector2d(18, fakey), Math.toRadians(0))
                                        .waitSeconds(0.1)
                                        .setReversed(false)
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(otherside)
                .start();
    }
}