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
@Autonomous(name = "RedHumanSide", group = "Autonomous")
public class RedHumanSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(15, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
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
                .strafeToLinearHeading(new Vector2d(42,-50), Math.toRadians(90));

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
