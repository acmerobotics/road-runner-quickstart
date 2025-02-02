package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Fourbar;
import org.firstinspires.ftc.teamcode.Common.Lift;
import org.firstinspires.ftc.teamcode.Common.Rotation;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTO Testing", group = "Autonomous")
public class AutoTesting extends LinearOpMode {

    private Limelight3A limelight;

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-8, -61, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-60)
                .splineToLinearHeading(new Pose2d(5, -50, Math.toRadians(70)), Math.toRadians(0));

        limelight.start();

        waitForStart();

        if (isStopRequested()) return;

        Action toBasket = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        toBasket
                )
        );

        if (limelight.getLatestResult().isValid()) {
            Pose3D result = limelight.getLatestResult().getBotpose();
            Pose2d endPose = new Pose2d(result.getPosition().x, result.getPosition().y, Math.toRadians(result.getOrientation().getYaw()));
            TrajectoryActionBuilder correction = drive.actionBuilder(endPose)
                    .splineToLinearHeading(new Pose2d(5, -40, Math.toRadians(70)), Math.toRadians(70));
            Action correctionA = correction.build();

            Actions.runBlocking(
                    new SequentialAction(
                            correctionA
                    )
            );
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