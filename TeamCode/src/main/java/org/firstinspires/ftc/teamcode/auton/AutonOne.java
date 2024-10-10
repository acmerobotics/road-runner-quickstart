package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AutonOne", group = "Autonomous")
public class AutonOne extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position of the robot (x = -11.8, y = -61.7, heading = -90 degrees)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        
        // Define trajectory using Pose2d for simultaneous right and forward movement
        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(0,0,0))
                // Move to a new pose 24 inches right and 12 inches forward
                .strafeTo(new Vector2d(24, 12)) // New position (x + 24, y + 12)
                // Move back 10 inches (y - 10)
                .strafeTo(new Vector2d(24, 2))
                // Strafe left 30 inches (x - 30)
                .strafeTo(new Vector2d(-6, 2));

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
