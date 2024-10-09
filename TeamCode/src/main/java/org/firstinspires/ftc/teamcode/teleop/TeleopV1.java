package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.setTelemToDashboard(telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.arcadeDriveWithSlowMode(gamepad1);
            bot.slideControl(gamepad2);
            bot.tiltControl(gamepad2);
            bot.wristControl(gamepad1);
//            bot.gripClawControl(gamepad2);
//            bot.intakeControl(gamepad2);
            bot.intakeOpen(gamepad2);

            bot.updateAxonPositions();

            telemetry.addData("FlipPos", bot.flip.getCurrentPosition());
            telemetry.addData("SlidePos", bot.slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
