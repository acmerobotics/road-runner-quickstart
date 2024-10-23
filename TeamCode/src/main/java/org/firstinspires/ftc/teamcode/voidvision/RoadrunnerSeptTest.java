package org.firstinspires.ftc.teamcode.voidvision;

import androidx.annotation.NonNull;

// Road Runner-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-Road Runner imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Autonomous mode for Roadrunner test.
 * Uses Roadrunner commands for trajectory building and executing actions.
 */
@Config
@Autonomous(name = "RoadrunnerSeptTest", group = "Autonomous")
public class RoadrunnerSeptTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the Mecanum drive at the start pose (0,0) and 0 radians rotation
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));

        // Create a Road Runner trajectory with two segments
        Action trajectoryAction1;
        trajectoryAction1 = drive.actionBuilder(drive.pose)
                .setTangent(0)         // Start with no angular rotation
                .lineToX(20)           // Move along the X-axis by 20 units
                .waitSeconds(4)        // Wait for 4 seconds at the end of the path
                .setTangent(Math.PI / 2) // Change the tangent (rotation) to 90 degrees (π/2 radians)
                .lineToY(20)           // Move along the Y-axis by 20 units
                .build();              // Build the trajectory action

        // Wait for start while showing telemetry
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        // Start the autonomous routine
        waitForStart();
        if (isStopRequested()) return;

        // Execute the built trajectory
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction1  // Run the first trajectory
                        // Add more trajectories/actions here if needed
                )
        );

        // Display current robot position for debugging/telemetry
        telemetry.addData("X", drive.pose);
    }
}
