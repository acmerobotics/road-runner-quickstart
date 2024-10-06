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
        // Starting position of the robot
        Pose2d initialPose = new Pose2d(-11.8, -61.7, Math.toRadians(-90)); // (x, y, heading)
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Vision system outputs position (set manually here for now)
        int visionOutputPosition = 1;

        // Define trajectory using Pose2d and Vector2d
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-11.8 + 24, -61.7)) // Strafe 24 inches to the right (new x = initial x + 24)
                .strafeTo(new Vector2d(-11.8 + 24, -61.7 + 12)) // Move forward 12 inches (new y = initial y + 12)
                .strafeTo(new Vector2d(-11.8 + 24, -61.7 + 12 - 10)) // Move back 10 inches (y - 10)
                .strafeTo(new Vector2d(-11.8 + 24 - 30, -61.7 + 12 - 10)); // Strafe left 30 inches (x - 30)

        // Final action to close out the trajectory
        Action trajectoryActionCloseOut = tab1.fresh().build();

        // Initialize loop to check vision output
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

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
