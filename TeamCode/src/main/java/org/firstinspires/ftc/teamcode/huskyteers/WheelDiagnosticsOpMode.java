package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(name = "Wheel Diagnostics OpMode", group = "Teleop")
public class WheelDiagnosticsOpMode extends LinearOpMode {
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
            float leftFront=0;
            float leftBack= 0;
            float rightFront=0;
            float rightBack=0;

            if (currentGamepad1.a) {
                leftFront = 1;
            }  if (currentGamepad1.b) {
                leftBack = 1;
            }  if (currentGamepad1.x) {
                rightFront = 1;
            } if (currentGamepad1.y) {
                rightBack = 1;
            }
            huskyBot.setMotorPowers(leftBack, leftFront, rightBack, rightFront);
            

            telemetry.addData("Left Bumper", currentGamepad1.left_bumper);
            telemetry.addData("April Detected", huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(583).isPresent());
            telemetry.update();
        }
    }
    // endregion
}
