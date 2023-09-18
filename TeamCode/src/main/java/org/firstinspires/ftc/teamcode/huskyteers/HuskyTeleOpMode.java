package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Husky TeleOp Mode" , group = "Teleop")
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
            if(gamepad1.left_bumper){
              PoseVelocity2d pw = huskyBot.alignWithAprilTag(1);
                huskyBot.driveRobot(pw.component1().y,pw.component1().x,pw.component2(),1.0);
            }
            else {

                huskyBot.fieldCentricDriveRobot(
                        -currentGamepad1.left_stick_y,
                        currentGamepad1.left_stick_x,
                        currentGamepad1.right_stick_x,
                        (0.35 + 0.5 * currentGamepad1.left_trigger));
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .


            if(huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag() != null)
                telemetry.addData("Closest April Tag Range", huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag().ftcPose.range);

            if(huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1) != null)
                telemetry.addData("April Tag ID 1:", huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1).ftcPose.range);

            telemetry.update();
        }
        // endregion

    }

}
