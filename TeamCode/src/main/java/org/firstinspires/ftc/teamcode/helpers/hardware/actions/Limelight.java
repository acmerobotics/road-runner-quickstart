package org.firstinspires.ftc.teamcode.helpers.hardware.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
                    cancelableTrajectoryAction.cancelAbruptly();
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
                    // Retrieve the target offset from Limelight (x, y)
                    Vector2d averagePose = motorControl.limelight.getAveragePoseInInches();

                    double adjustedX = averagePose.x * -39.3700787 * 1.5 + 0.5;
                    double adjustedY = averagePose.y * 39.3700787 * 0.8 + 2;

                    moveToTargetAction = drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(adjustedX + drive.pose.position.x, adjustedY + drive.pose.position.y))
                            .build();

                    moveToTargetStarted = true; // **Fixed: Set flag to true to initiate movement**

                    telemetry.addData("Average X", adjustedX);
                    telemetry.addData("New X", adjustedX + drive.pose.position.x);
                    telemetry.addData("Average Y", adjustedY);
                    telemetry.addData("New Y", adjustedY + drive.pose.position.y);
                    telemetry.update();

                    // Optionally reset samplesStarted to allow new sample collection after moving
                    samplesStarted = false;

                    return true; // Continue running
                } else {
                    // Continue monitoring
                    return true;
                }
            }
        }
    };
}
