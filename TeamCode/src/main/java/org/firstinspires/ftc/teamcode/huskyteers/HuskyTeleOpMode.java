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

        Gamepad currentGamepad1;
        Gamepad currentGamepad2;

        waitForStart();
        if (isStopRequested()) return;
        // endregion

        // region TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

            huskyBot.fieldCentricDriveRobot(
                    -currentGamepad1.left_stick_y,
                    currentGamepad1.left_stick_x,
                    currentGamepad1.right_stick_x,
                    (0.35 + 0.5 * currentGamepad1.left_trigger));


            if(huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag() != null)
                telemetry.addData("Closest April Tag Range", huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag().get().ftcPose.range);

            if (huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1) != null) {
                if (huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1).get().ftcPose == null) {
                    telemetry.addData("Error: ", "April tag with ID 1 has no pose");
                } else {
                    telemetry.addData("April Tag ID 1:", huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1).get().ftcPose.range);
                }
            }
            telemetry.update();
        }
        // endregion
    }

}
