package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
@TeleOp(name = "Husky TeleOp Mode", group = "Teleop")
public class HuskyTeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // region INITIALIZATION
        HuskyBot huskyBot = new HuskyBot(this);
        GamepadUtils gamepadUtils = new GamepadUtils();
        huskyBot.init();



        waitForStart();
        if (isStopRequested()) return;
        // endregion
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        AtomicBoolean usingFieldCentric = new AtomicBoolean(true);
        gamepadUtils.addRisingEdge("a", d -> {
            usingFieldCentric.set(!usingFieldCentric.get());
        });

        // region TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            gamepadUtils.processUpdates(currentGamepad1);

            if (currentGamepad1.start) {
                huskyBot.setCurrentHeadingAsForward();
            }

            if (currentGamepad1.left_bumper &&
                    huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(583).isPresent()) {
                PoseVelocity2d pw = huskyBot.alignWithAprilTag(583);
                TelemetryUtils.PoseVelocity2d(pw);
                huskyBot.driveRobot(pw.component1().y, pw.component1().x, pw.component2(), 1.0);
            } else {
                if (usingFieldCentric.get()) {
                    huskyBot.fieldCentricDriveRobot(
                            -currentGamepad1.left_stick_y,
                            currentGamepad1.left_stick_x,
                            currentGamepad1.right_stick_x,
                            (0.35 + 0.5 * currentGamepad1.left_trigger));
                } else {
                    huskyBot.driveRobot(
                            -currentGamepad1.left_stick_y,
                            currentGamepad1.left_stick_x,
                            currentGamepad1.right_stick_y,
                            (0.35 + 0.5 * currentGamepad1.left_trigger));
                }
            }


            huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(583).ifPresent(
                    TelemetryUtils::AprilTagDetection);
            telemetry.update();
        }
    }
    // endregion
}
