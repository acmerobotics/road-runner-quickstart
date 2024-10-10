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
@Autonomous(name = "AutonOne", group = "Autonomous")
public class RedHumanSide extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(15, 63, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(15,63,270))
                .strafeTo(new Vector2d(-11,58))
                .waitSeconds(2.5)
                .strafeToLinearHeading(new Vector2d(-56,48), Math.toRadians(295))
                .waitSeconds(3)
                .turn(Math.toRadians(-30))
                .waitSeconds(1)
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .turn(Math.toRadians(-55))
                .waitSeconds(1)
                .turn(Math.toRadians(55))
                .waitSeconds(2)
                .turn(Math.toRadians(-25))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56,48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11,58))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56,48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11,58))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-56,48))
                .waitSeconds(2)
                .strafeTo(new Vector2d(-11,58));

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
