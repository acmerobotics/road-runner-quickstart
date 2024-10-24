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
@Autonomous(name = "BlueHumanSide", group = "Autonomous")
public class BlueHumanSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(-15, 63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        
        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-11,48))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(-30,48), Math.toRadians(255))
                .strafeToLinearHeading(new Vector2d(-38,11), Math.toRadians(260))
                .strafeToLinearHeading(new Vector2d(-45,11), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-45,54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-45,11), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,11), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-56,11), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-61,11), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-61,54), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-48,55), Math.toRadians(270))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-4,45), Math.toRadians(270))
                .waitSeconds(1.86)
                .strafeTo(new Vector2d(-48,54))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(-4,45))
                .waitSeconds(1.86)
                .strafeToLinearHeading(new Vector2d(-42,52), Math.toRadians(270));

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
