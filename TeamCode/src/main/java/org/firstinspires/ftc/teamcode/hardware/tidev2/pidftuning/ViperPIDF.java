package org.firstinspires.ftc.teamcode.hardware.tidev2.pidftuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ViperPIDF extends OpMode {
    private PIDFController controller;

    public static double p = 0.001, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 360.0;


    private DcMotorEx viper;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        viper = hardwareMap.get(DcMotorEx.class, "viper_slide");

        viper.setDirection(DcMotorSimple.Direction.FORWARD);

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        int vipPos = viper.getCurrentPosition();
        double pidf = controller.calculate(vipPos, target);
        // target limits: 0, 5500

        viper.setPower(pidf);

        telemetry.addData("pidf", pidf);
        telemetry.addData("pos", vipPos);
        telemetry.addData("target", target);
    }
}
