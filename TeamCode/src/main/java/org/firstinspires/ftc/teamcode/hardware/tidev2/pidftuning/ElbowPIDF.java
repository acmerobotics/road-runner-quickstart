package org.firstinspires.ftc.teamcode.hardware.tidev2.pidftuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp
public class ElbowPIDF extends OpMode {
    private PIDFController controller;

    public static double p = 0.001, i = 0.0001, d = 0.0001;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 360.0;


    private DcMotorEx elbow;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        int elbPos = elbow.getCurrentPosition();
        double pidf = controller.calculate(elbPos, target);


        elbow.setPower(pidf);

        telemetry.addData("pidf", pidf);
        telemetry.addData("pos", elbPos);
        telemetry.addData("target", target);
    }
}
