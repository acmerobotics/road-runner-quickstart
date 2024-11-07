package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp RED Main")
@Config
public class TeleOpMainRed extends LinearOpMode {
    double oldTime = 0;
    AllianceColor allianceColor = AllianceColor.RED;

    // STATES
    boolean manualExtension = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);
        List<Action> actionsQueue = new ArrayList<>();
        List<Action> runningActions = new ArrayList<>();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            if (gamepad1.left_bumper) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(robot::teleDepositPreset)
                ));
            }
            if (gamepad1.right_bumper) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(robot::intakePreset)
                ));
            }
            if (gamepad1.triangle) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(robot::toggleGamepiece)
                ));
            }
            if (gamepad1.square) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(this::setManualExtension)
                ));
            }
            if (gamepad1.circle) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(() -> robot.toggleGamepieceColor(allianceColor))
                ));
            }
            if (gamepad1.cross) {
                actionsQueue.add(new SequentialAction(
                        new InstantAction(robot::smartOuttake)
                ));
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : actionsQueue) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("looptime: ", frequency);
            telemetry.update();

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            robot.setDrivePower(-x, y, rx);

        }
    }

    private void setManualExtension() {
        manualExtension = !manualExtension;
    }
}
