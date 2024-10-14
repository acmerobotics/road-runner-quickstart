package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RedBasketSideSample", group = "Autonomous")
public class RedBasketSideSample extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(-39, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-58,-48), Math.toRadians(65))
                .waitSeconds(3.5)
                .turn(Math.toRadians(30))
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .waitSeconds(2)
                .turn(Math.toRadians(55))
                .waitSeconds(1)
                .turn(Math.toRadians(-55))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-42,-9), Math.toRadians(0))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(-58,-48), Math.toRadians(65))
                .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(-42,-9), Math.toRadians(0))
                .waitSeconds(3.5)
                .strafeToLinearHeading(new Vector2d(-58,-48), Math.toRadians(65));


        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        // Wait for the start of the op mode
        waitForStart();
        if (isStopRequested()) return;

        // Execute the defined trajectory
        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}
