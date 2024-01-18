package org.firstinspires.ftc.teamcode.MainCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class VelocityTest extends LinearOpMode {

    DcMotorEx motor;

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int  target = 0;
    private final double ticks_in_degree = 144 / 180;
    public static double ArmMotorOffset = 18;
    public static double SecondArmMotorOffset = -18;


    @Override


    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        while (opModeIsActive()) {


        }

    }

}
