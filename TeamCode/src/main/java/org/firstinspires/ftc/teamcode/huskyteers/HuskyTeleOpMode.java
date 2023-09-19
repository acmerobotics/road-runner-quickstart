package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Husky TeleOp Mode", group = "Teleop")
public class HuskyTeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {

        // region INITIALIZATION
        HuskyBot huskyBot = new HuskyBot(this);
        huskyBot.init();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;
        // endregion

        // region TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

            PoseVelocity2d pw = huskyBot.alignWithAprilTag(1);
            huskyBot.driveRobot(pw.component1().y, pw.component1().x, -pw.component2(), 1.0);

            telemetry.addData("Drive", pw.component1().y);
            telemetry.addData("Strafe", pw.component1().x);
            telemetry.addData("Turn", pw.component2());

            if (huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag() != null) {
                if (huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag().ftcPose == null) {
                    telemetry.addData("Error: ", "April tag has no pose");
                } else {
                    telemetry.addData("Closest April Tag Range", huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag().ftcPose.range);
                }
            }
            if (huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1) != null) {
                if (huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag().ftcPose == null) {
                    telemetry.addData("Error: ", "April tag has no pose");
                } else
                    telemetry.addData("April Tag ID 1:", huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1).ftcPose.range);
            }
            telemetry.update();
        }
        // endregion
    }

}
