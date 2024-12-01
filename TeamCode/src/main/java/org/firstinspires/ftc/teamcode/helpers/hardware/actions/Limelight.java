package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;

public class Limelight {
    private final PinpointDrive.CancelableTrajectoryAction cancelableTrajectoryAction;
    private final MotorControl motorControl;
    private final PinpointDrive drive;
    private final Telemetry telemetry;

    private boolean samplesStarted = false;

    private boolean moveToTargetStarted = false;
    private Action moveToTargetAction;
    private boolean actionComplete = false;

    public Limelight(PinpointDrive.CancelableTrajectoryAction cancelableTrajectoryAction, MotorControl motorControl, PinpointDrive drive, Telemetry telemetry) {
        this.cancelableTrajectoryAction = cancelableTrajectoryAction;
        this.motorControl = motorControl;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    public Action limelightMonitorAction = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (moveToTargetStarted) {
                // Continue running the moveToTargetAction
                if (moveToTargetAction != null) {
                    boolean moveToTargetInProgress = moveToTargetAction.run(packet);
                    if (!moveToTargetInProgress) {
                        // Movement to target is complete, reset flags to start monitoring again
                        moveToTargetStarted = false;
                        moveToTargetAction = null;
                        actionComplete = true; // Set this if you want to stop monitoring after reaching the target
                    }
                    return !actionComplete; // Continue running if not complete
                } else {
                    // No moveToTargetAction defined, start monitoring again
                    moveToTargetStarted = false;
                    return true;
                }
            } else {
                // Monitor Limelight data

                if (!samplesStarted) {
                    motorControl.limelight.startCollectingSamples();
                    samplesStarted = true;
                }

                boolean samplesCollected = motorControl.limelight.collectSamples();

                if (samplesCollected) {
                    // Retrieve the target offset from Limelight (assumed to be in inches)
                    Vector2d targetOffset = motorControl.limelight.getAveragePoseInInches();

                    // Get current robot pose
                    Pose2d currentPose = drive.pose;

                    // Extract current heading in radians
                    double headingRadians = currentPose.heading.toDouble();

                    // Calculate the cosine and sine of the heading
                    double cosHeading = Math.cos(headingRadians);
                    double sinHeading = Math.sin(headingRadians);

                    // Correct Rotation Formula
                    double rotatedX = targetOffset.x * cosHeading - targetOffset.y * sinHeading;
                    double rotatedY = targetOffset.x * sinHeading + targetOffset.y * cosHeading;

                    // Calculate the new target position by adding the rotated offset to current position
                    double targetX = currentPose.position.x + rotatedX;
                    double targetY = currentPose.position.y + rotatedY;

                    // Define the new target heading
                    Rotation2d targetHeading = currentPose.heading; // Keeping current heading

                    // Create the new target position vector
                    Vector2d targetPosition = new Vector2d(targetX, targetY);

                    // Debugging: Print out the computed values
                    telemetry.addData("Current Pose", String.format("(%.2f, %.2f, %.2f°)", currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble())));
                    telemetry.addData("Limelight X (in)", String.format("%.4f", targetOffset.x));
                    telemetry.addData("Limelight Y (in)", String.format("%.4f", targetOffset.y));
                    telemetry.addData("Rotated Offset (Global)", String.format("(%.2f, %.2f)", rotatedX, rotatedY));
                    telemetry.addData("New Target Position", String.format("(%.2f, %.2f)", targetX, targetY));
                    telemetry.addData("Target Heading", String.format("%.2f°", Math.toDegrees(targetHeading.toDouble())));
                    telemetry.update();

                    // Build a new trajectory to the target using strafeToLinearHeading
                    moveToTargetAction = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(targetPosition, targetHeading)
                            .build();

                    moveToTargetStarted = true;
                    return true; // Continue running
                } else {
                    // Continue monitoring
                    return true;
                }
            }
        }
    };
}
