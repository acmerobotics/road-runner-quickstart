package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
@TeleOp(name = "Husky TeleOp Mode", group = "Teleop")
public class HuskyTeleOpMode extends HuskyBot {
    @Override
    public void runOpMode() {
        // region INITIALIZATION
        super.initializeHardware();
        GamepadUtils gamepadUtils = new GamepadUtils();


        waitForStart();
        if (isStopRequested()) return;
        // endregion
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        AtomicBoolean usingFieldCentric = new AtomicBoolean(true);
        gamepadUtils.addRisingEdge("a", d -> {
            usingFieldCentric.set(!usingFieldCentric.get());
            gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder().addStep(1, 1, 200).build());
        });

        // region TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            gamepadUtils.processUpdates(currentGamepad1);

            if (currentGamepad1.start) {
                setCurrentHeadingAsForward();
            }

            if (currentGamepad1.left_bumper &&
                    huskyVision.AprilTagDetector.getAprilTagById(583).isPresent()) {
                PoseVelocity2d pw = alignWithAprilTag(583);
                TelemetryUtils.PoseVelocity2d(pw);
                driveRobot(pw.component1().y, pw.component1().x, pw.component2(), 1.0);
            } else {
                if (usingFieldCentric.get()) {
                    telemetry.addLine("Currently using field centric");
                    fieldCentricDriveRobot(
                            currentGamepad1.left_stick_y,
                            -currentGamepad1.left_stick_x,
                            currentGamepad1.right_stick_x,
                            (0.35 + 0.5 * currentGamepad1.left_trigger));
                } else {
                    telemetry.addLine("Currently using tank drive");
                    driveRobot(
                            currentGamepad1.left_stick_y,
                            -currentGamepad1.left_stick_x,
                            currentGamepad1.right_stick_x,
                            (0.35 + 0.5 * currentGamepad1.left_trigger));
                }
            }

            huskyVision.AprilTagDetector.getAprilTagById(583).ifPresent(
                    TelemetryUtils::AprilTagDetection);
            TelemetryUtils.Gamepad(currentGamepad1);
            telemetry.update();
        }
    }
    // endregion
}
