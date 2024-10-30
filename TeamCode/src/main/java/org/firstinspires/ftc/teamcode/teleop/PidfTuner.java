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
public class PidfTuner extends OpMode {
    private PIDController armController, slideController;

    public static double fP = 0.0018, fI = 0, fD = 0.00009;  //fD = 0.00001, fP = 0.002
    public static double fF = 0.0037; //fF = 0.0022
    public static double sP = 0.005, sI, sD;
    public static double sF;

    public static int armTarget = 500;
    public static int slideTarget = 500;
    public static double servoTarget = 0.5;

    public static double multiplier = 0.5;

    private final double ticks_in_degree = 2048 / 90.0;

    private DcMotorEx flip, slide;
    private Servo wrist;

    @Override
    public void init() {
        Robot bot = new Robot(hardwareMap);

        armController = new PIDController(fP, fI, fD);
        slideController = new PIDController(sP,sI,sD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flip = (DcMotorEx) bot.flip;
        slide = (DcMotorEx) bot.slide;
        wrist = bot.wrist;

        flip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flip.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        int armPos, slidePos;

        armController.setPID(fP, fI, fD);
        armPos = flip.getCurrentPosition();
        double pid = armController.calculate(armPos, armTarget);
        double ff = Math.cos(Math.toRadians(armTarget / ticks_in_degree)) * fF;

        double power = pid + ff;
        if (Math.abs(armTarget-armPos)<100){
            multiplier=0.001;
        }

        flip.setPower(power * multiplier);

        slideController.setPID(sP,sI,sD);
        slidePos = slide.getCurrentPosition();
        double pid2 = slideController.calculate(slidePos, slideTarget);

        slide.setPower(pid2);

        wrist.setPosition(servoTarget);

        telemetry.addData("armPos", armPos);
        telemetry.addData("flipPower", power);
        telemetry.addData("slidePos", slidePos);
        telemetry.addData("slidePower", slide.getPower());
        telemetry.update();
    }

}

