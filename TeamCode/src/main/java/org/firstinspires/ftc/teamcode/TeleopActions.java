package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.data.PositionsClass;
import org.firstinspires.ftc.teamcode.helpers.hardware.MotorControl;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.hardware.actions.MotorActions;

@TeleOp(name = "Actions Teleop")
@Config
public class TeleopActions extends ActionOpMode {

    MotorControl motorControl;
    MotorActions motorActions;

    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;

    private boolean overRideSpecimen = false;
    private boolean gamepad2aPressed = false;




    private boolean specimenArea = false;

    @Override
    public void runOpMode() {
        Pose2d startingPose = org.firstinspires.ftc.teamcode.helpers.data.PoseStorage.loadPose();
        if (startingPose == null) {
            startingPose = new Pose2d(0, 0, 0);
            telemetry.addData("No saved pose found", "Starting at default pose");
            telemetry.update();
        } else {
            telemetry.addData("Loaded saved pose", startingPose.toString());
            telemetry.update();
        }

        PinpointDrive drive = new PinpointDrive(hardwareMap, startingPose);

        motorControl = new MotorControl(hardwareMap, telemetry);
        motorActions = new MotorActions(motorControl, drive);



        waitForStart();

        if (isStopRequested()) return;

        run(motorActions.extendo.findZero());
        run(new ParallelAction(
                motorActions.intakeTransfer(),
                motorActions.outtakeTransfer()
        ));
        while (opModeIsActive() && !isStopRequested()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.right_stick_y,
                            -gamepad1.right_stick_x
                    ),
                    -gamepad1.left_stick_x
            ));

            // Existing controls for intake extension and transfer
            if (gamepad1.right_bumper && motorActions.intakePosition != PositionsClass.Intake.Extended) {
                run(motorActions.intakeExtend(600, 2));
            }
            else if (gamepad1.left_bumper&& motorActions.intakePosition != PositionsClass.Intake.Transfer) {
                run(motorActions.intakeTransfer());
            }

            if (gamepad2.a && !gamepad2aPressed) {
                overRideSpecimen = !overRideSpecimen;
                gamepad2aPressed = true;
            } else if (!gamepad2.a) {
                gamepad2aPressed = false;
            }



            // Use intakePosition state to check if the intake is extended
            if (motorActions.intakePosition == PositionsClass.Intake.Extended) {
                // Triggers control the intake turret
                if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                    leftTriggerPressed = true;
                    run(motorActions.intakeTurret.moveLeft());
                } else if (gamepad1.left_trigger == 0) {
                    leftTriggerPressed = false;
                }

                if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                    rightTriggerPressed = true;
                    run(motorActions.intakeTurret.moveRight());
                } else if (gamepad1.right_trigger == 0) {
                    rightTriggerPressed = false;
                }
            } else {
                // Triggers control the lift as before
                if (gamepad1.right_trigger > 0) {
                    if ( overRideSpecimen){
                        run(motorActions.outtakeDeposit());
                    }
                    else{
                        run(motorActions.outtakeSpecimen());
                    }
                } else if (gamepad1.left_trigger > 0) {
                    if (motorActions.outtakePosition == PositionsClass.OutTake.Specimen){
                        run(new SequentialAction(
                                motorActions.depositeSpecimen(),
                                motorActions.outtakeTransfer()
                        ));
                    }
                    else {
                        run(motorActions.outtakeTransfer());
                    }

                }
            }

            TelemetryPacket packet = new TelemetryPacket();
            updateAsync(packet);
            motorControl.update();
            drive.updatePoseEstimate();
            // Pose
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Debug
            telemetry.addData("Intake Position State:", motorActions.intakePosition);
            telemetry.addData("Extendo Position:", motorControl.extendo.motor.getCurrentPosition());
            telemetry.addData("Extendo Finished:", motorControl.extendo.closeEnough());
            telemetry.addData("Extendo Resetting:", motorControl.extendo.isResetting());
            telemetry.addData("Intake Arm Position:", motorControl.intakeLeft.getCurrentPosition());
            telemetry.addData("Intake Arm Finished:", motorControl.intakeLeft.closeEnough());
            telemetry.addData("Intake Turret Position Index:", motorActions.intakeTurret.currentPositionIndex);
            telemetry.addData("Specimen Override:", overRideSpecimen ? "Enabled" : "Disabled");
            telemetry.addData("In Specimen Area:", specimenArea);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
