package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopForLittleChildren extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.arcadeDriveWithSlowModeForLittleChildren(gamepad1);

            telemetry.addData("FlipPos", bot.flip.getCurrentPosition());
            telemetry.addData("SlidePos", bot.slide.getCurrentPosition());
            telemetry.addData("wristPos", bot.wrist.getPosition());
            telemetry.addData("flipTarget", bot.armTarget);
            telemetry.addData("slideTarget", bot.slideTarget);
            telemetry.addData("fliPower", bot.flip.getPower());
            telemetry.addData("slidePower", bot.slide.getPower());
            telemetry.update();
        }
    }
}
