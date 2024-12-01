package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.data.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionHelpers;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;

@Autonomous(name = "BucketTestActionAuto")
public final class Bucket extends LinearOpMode {

    MotorControl motorControl;
    MotorActions motorActions;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-16, -62, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        motorControl = new MotorControl(hardwareMap, telemetry);
        motorActions = new MotorActions(motorControl, drive);

        waitForStart();

        if (isStopRequested()) return;

        // Build the drive action
        Action driveAction = drive.actionBuilder(beginPose)
                .afterDisp(1, new SequentialAction(
                        motorActions.outTakeClaw.Close(),
                        motorActions.intakeTurret.setPositionByIndex(2),
                        motorActions.outtakeSpecimen()
                )) // Slap on specimen
                .strafeTo(new Vector2d(-7, -31.2)) // Go to truss
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.5),
                        motorActions.depositeSpecimen(),
                        new SleepAction(0.2),
                        motorActions.outtakeTransfer(),
                        motorActions.extendo.findZero()
                ))
                .afterDisp(40, motorActions.intakeExtend(550, 2))
                .strafeToLinearHeading(new Vector2d(-51, -52), Math.toRadians(90)) // First yellow
                .stopAndAdd(new SequentialAction(
                        motorActions.extendo.waitUntilFinished(),
                        motorActions.intakeTransfer(),
                        motorActions.outtakeTransfer())
                )
                .strafeToLinearHeading(new Vector2d(-59,-55.5), Math.toRadians(45)) // Basket
                .stopAndAdd(new SequentialAction(
                        motorActions.outtakeDeposit(),
                        motorActions.outtakeTransfer()))
                .afterDisp(1, motorActions.intakeExtend(520, 2))
                .strafeToLinearHeading(new Vector2d(-59,-51), Math.toRadians(95))// Second Yellow
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.2),
                        motorActions.intakeTransfer(),
                        motorActions.outtakeTransfer())
                )
                .strafeToLinearHeading(new Vector2d(-59.5,-55.5), Math.toRadians(45)) // Basket
                .stopAndAdd(new SequentialAction(
                        motorActions.outtakeDeposit(),
                        motorActions.outtakeTransfer()))
                .strafeToLinearHeading(new Vector2d(-51.5,-44), Math.toRadians(135))// Third Yellow
                .stopAndAdd(new SequentialAction(
                        motorActions.intakeExtend(600, 3),
                        new SleepAction(0.2),
                        new ParallelAction(
                                motorActions.intakeTransfer()
                        ),
                        motorActions.outtakeTransfer())
                )
                .strafeToLinearHeading(new Vector2d(-59.5,-55.5), Math.toRadians(45)) // Basket
                .stopAndAdd(new SequentialAction(
                        motorActions.outtakeDeposit(),
                        motorActions.outtakeTransfer()))
                .build();

        // Run the drive action and motor actions update in parallel until the drive action finishes
        Actions.runBlocking(new ActionHelpers.RaceParallelCommand(
                driveAction,
                motorActions.update()
        ));

        // Save the robot's pose at the end of Autonomous
        Pose2d finalPose = drive.pose;
        PoseStorage.savePose(finalPose);
        telemetry.addData("Pose Saved", finalPose.toString());
        telemetry.update();
    }
}
