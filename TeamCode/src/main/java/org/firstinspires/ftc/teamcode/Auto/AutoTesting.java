package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Limelight;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTO Testing", group = "Autonomous")
public class AutoTesting extends LinearOpMode {

    private Limelight3A limelight;
    private Extension extension;

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-8, -61, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        extension = new Extension(hardwareMap);

        Pose2d toBasket_lastPose = new Pose2d(0, -35, Math.toRadians(90));
        Pose2d pushSample1_lastPose = new Pose2d(45, -40, Math.toRadians(180));
        Pose2d pushSample2_lastPose = new Pose2d(54, -40, Math.toRadians(180));
        Pose2d pushSample3_lastPose = new Pose2d(35, -40, Math.toRadians(180));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-60)
                .splineToLinearHeading(toBasket_lastPose, Math.toRadians(0));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(180)), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 100)) // new ProfileAccelConstraint(-100, 100)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(36, -4, Math.toRadians(180)), Math.toRadians(90), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45,  -4, Math.toRadians(180)), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(45, -44, Math.toRadians(180)), Math.toRadians(270), null, new ProfileAccelConstraint(-100, 100))
                .splineToLinearHeading(pushSample1_lastPose, Math.toRadians(270));

        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(43, -4, Math.toRadians(180)), Math.toRadians(90), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54,  -4, Math.toRadians(180)), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(54, -44, Math.toRadians(180)), Math.toRadians(270), null, new ProfileAccelConstraint(-100, 100))
                .splineToLinearHeading(pushSample2_lastPose, Math.toRadians(270));

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, -4, Math.toRadians(180)), Math.toRadians(90), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(64,  -4, Math.toRadians(180)), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(64, -44, Math.toRadians(180)), Math.toRadians(270), null, new ProfileAccelConstraint(-100, 100))
                .splineToLinearHeading(new Pose2d(64, -40, Math.toRadians(180)), Math.toRadians(270))
                .splineToLinearHeading(pushSample3_lastPose, Math.toRadians(270));

        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .turnTo(270);

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(270));

        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(270));

        limelight.start();

        waitForStart();

        if (isStopRequested()) return;

        Action toBasket = tab1.build();
        Action pushSample1 = tab2.build();
        Action pushSample2 = tab3.build();
        Action pushSample3 = tab4.build();

        Actions.runBlocking(
                new SequentialAction(
                        extension.Retract(),
                        toBasket,
                        new SleepAction(5)
                )
        );

        aprilTagCorrection(toBasket_lastPose, drive, limelight);

        Actions.runBlocking(
                new SequentialAction(
                        pushSample1,
                        pushSample2,
                        pushSample3
                )
        );

        Pose3D result = limelight.getLatestResult().getBotpose();
        Pose2d actualPose = new Pose2d(Limelight.metersToInches(result.getPosition().x), Limelight.metersToInches(result.getPosition().y), Math.toRadians(result.getOrientation().getYaw()));

        Action tab8 = drive.actionBuilder(actualPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(43, -4, Math.toRadians(180)), Math.toRadians(90), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54,  -4, Math.toRadians(180)), Math.toRadians(0), null, new ProfileAccelConstraint(-100, 100))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(54, -44, Math.toRadians(180)), Math.toRadians(270), null, new ProfileAccelConstraint(-100, 100))
                .splineToLinearHeading(pushSample2_lastPose, Math.toRadians(270))
                .build();

        aprilTagCorrection(pushSample3_lastPose, drive, limelight);
    }

    public void aprilTagCorrection(Pose2d correctionPose, MecanumDrive MDrive, Limelight3A limelight) {
        if (limelight.getLatestResult() != null && limelight.getLatestResult().isValid()) {
            telemetry.addData("Result is: ", limelight.getLatestResult().isValid());
            telemetry.addData("Bot Pose in Pose3d is: ", limelight.getLatestResult().getBotpose().toString());

            Pose3D result = limelight.getLatestResult().getBotpose();

            Pose2d endPose = new Pose2d(Limelight.metersToInches(result.getPosition().x), Limelight.metersToInches(result.getPosition().y), Math.toRadians(result.getOrientation().getYaw()));

            telemetry.addData("Bot Pose in Pose2d is: ", endPose.toString());
            telemetry.addData("X/Y error", Limelight.distanceBetweenPose(endPose, correctionPose));
            telemetry.addData("Heading error", Limelight.headingDifferencePose(endPose, correctionPose));
            telemetry.update();

            if (Limelight.distanceBetweenPose(endPose, correctionPose) > 2.5 || Limelight.headingDifferencePose(endPose, correctionPose) > 8){
                Action correctionA = MDrive.actionBuilder(endPose)
                        .splineToLinearHeading(correctionPose, Math.toRadians(270))
                        .build();

                Actions.runBlocking(
                        new SequentialAction(
                                correctionA
                        )
                );
            } else {
                telemetry.addLine("No Correction Needed!");
                telemetry.update();
            }
        }
    }
}
/*
spline 1(to chambers)

linear slides up

four bar set position 90 degrees

coaxial set position 90 degrees

linear slides down (not fully)

claw open

linear slides down fully

spline 2 (park)
 */