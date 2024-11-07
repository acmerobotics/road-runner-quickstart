package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot bot = new Robot(hardwareMap);
        bot.setTelemToDashboard(telemetry);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            bot.arcadeDriveWithSlowMode(gamepad1);
//            bot.slideControl(gamepad2);
//            bot.tiltControl(gamepad2);
           bot.TeleopPID(gamepad2);
//            bot.slidesPID(gamepad2);
            bot.wristControl(gamepad2);
            bot.intakeControl(gamepad2);
//            bot.scoringMacro(gamepad2);
            bot.intakeOpen(gamepad2);
            bot.hangControl(gamepad1);

//            bot.updateAxonPositions();

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
