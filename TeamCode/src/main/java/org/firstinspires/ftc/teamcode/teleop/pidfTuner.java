package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class pidfTuner extends OpMode {
    private PIDController controller;

    public static double fP, fI, fD;
    public static double fF;
    public static double sP, sI, sD;
    public static double sF;

    public static boolean slides = false;

    public static int target = 1000;

    private final double ticks_in_degree = 1753 / 180.0;

    private DcMotorEx flip, slide;

    @Override
    public void init() {
        Robot bot = new Robot(hardwareMap);

        if (!slides) {
            controller = new PIDController(fP, fI, fD);
        }
        else {
            controller = new PIDController(sP,sI,sD);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flip = (DcMotorEx) bot.flip;
        slide = (DcMotorEx) bot.slide;

        flip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flip.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        int armPos;

        if (!slides) {
            slide.setPower(0);

            controller.setPID(fP, fI, fD);
            armPos = flip.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * fF;

            double power = pid + ff;

            flip.setPower(power);
        }
        else {
            flip.setPower(0);

            controller.setPID(sP,sI,sD);
            armPos = slide.getCurrentPosition();
            double pid = controller.calculate(armPos, target);

            double power = pid + sF;

            slide.setPower(power);
        }

        telemetry.addData("armPos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }

}

