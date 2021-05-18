package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

public class MeepMeepTest {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep mm = new MeepMeep(800)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(99, 30, Math.toRadians(37.74), Math.toRadians(60), 37)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, 48, 0))
                                .lineTo(new Vector2d(56, 48))
                                .addTemporalMarker(15.0,() -> {
                                    // do stuff
                                })
//                                .lineToConstantHeading(new Vector2d(56, 48))
                                //drop wobble
//                                .lineToConstantHeading(new Vector2d(56, 19))
                                .addDisplacementMarker(() -> {
                                    // drop wobble goal
                                    //return Unit.INSTANCE;
                                })
                                .waitSeconds(1.5)
                                // Get to wobble goal
                                .lineToSplineHeading(new Pose2d(-12, 19, Math.toRadians(0)))
                                // slowly approach wobble goal
                                .lineTo(new Vector2d(-27, 19), new TranslationalVelocityConstraint(5))
                                // grab wobble goal
                                .addDisplacementMarker(() -> {
                                    // grab wobble goal
                                    //return Unit.INSTANCE;
                                })
                                .waitSeconds(2.0)
                                .strafeTo(new Vector2d(56, 19))
                                .lineToLinearHeading(new Pose2d(56, 36, Math.toRadians(-90)))
                                .addDisplacementMarker(() -> {
                                    // drop wobble goal
                                    //return Unit.INSTANCE;
                                })
                                .strafeTo(new Vector2d(56, 30))
                                .strafeTo(new Vector2d(12, 30))
                                .waitSeconds(1.0)
                                .build()
                )
                .start();

/*

                        .lineTo(new Vector2d(12, 48))
                        .splineToConstantHeading(new Vector2d(12, 19), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-18, 19), Math.toRadians(0))
                        .lineTo(
                        new Vector2d(-27, 19)
//                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineToConstantHeading(new Vector2d(12, 19), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(-90)), Math.toRadians(0))
                .start()
 */
    }
}