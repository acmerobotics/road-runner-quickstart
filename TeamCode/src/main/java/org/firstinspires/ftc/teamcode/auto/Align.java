package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.helpers.data.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionHelpers;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.Limelight;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;

@Autonomous(name = "Align Test")
public final class Align extends LinearOpMode {

    MotorControl motorControl;
    MotorActions motorActions;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        motorControl = new MotorControl(hardwareMap, telemetry);
        motorActions = new MotorActions(motorControl, drive); // Pass drive here



        waitForStart();

        if (isStopRequested()) return;

        // Initial trajectory action
        Action trajectoryAction = drive.actionBuilder(beginPose)
                .lineToY(1)
                .afterDisp(1, motorActions.intakeExtend(600, 2))
                .build();

        // Wrap it in a CancelableTrajectoryAction
        PinpointDrive.CancelableTrajectoryAction cancelableTrajectoryAction =
                new PinpointDrive.CancelableTrajectoryAction(trajectoryAction, drive);

        // Instantiate the Limelight action
        Limelight limelightAction = new Limelight(cancelableTrajectoryAction, motorControl, drive, telemetry);

        Action limelightMonitorAction = limelightAction.limelightMonitorAction;

        // Run the initial trajectory action, Limelight action, and motor actions update in parallel
        Actions.runBlocking(new ParallelAction(
                cancelableTrajectoryAction,
                limelightMonitorAction,
                motorActions.update()
        ));



        // Save the robot's pose at the end of Autonomous
        Pose2d finalPose = drive.pose;
        PoseStorage.savePose(finalPose);
        telemetry.addData("Pose Saved", finalPose.toString());
        telemetry.update();
    }
}

