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
public class ShoulderPIDF extends OpMode {
    private PIDFController controller;

    public static double p = 0.004, i = 0.0001, d = 0;
    public static double f = 0.003;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 360.0;


    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;
    private DcMotorEx viper;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shoulder_right = hardwareMap.get(DcMotorEx.class, "left_tower");
        shoulder_left = hardwareMap.get(DcMotorEx.class, "right_tower");
        viper = hardwareMap.get(DcMotorEx.class, "viper_slide");


        shoulder_right.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder_left.setDirection(DcMotorSimple.Direction.FORWARD);

        shoulder_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        int armPos = shoulder_left.getCurrentPosition();
        double pidf = controller.calculate(armPos, target);

        shoulder_right.setPower(pidf);
        shoulder_left.setPower(pidf);

        telemetry.addData("pidf", pidf);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
    }
}
