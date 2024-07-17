package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class TestConfigMotor1PID extends LinearOpMode {
    DcMotorEx motor;
    public static String name = "";
    public static double P = 0, I = 0, D = 0;
    public static int target = 0;
    public int currentPos;
    public PIDController pidController;
    @Override
    public void runOpMode() throws InterruptedException {
        pidController = new PIDController(P, I, D);
        pidController.maxOutput = 0.8;
        motor = hardwareMap.get(DcMotorEx.class, name);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive())
        {
            pidController.p = P;
            pidController.i = I;
            pidController.d = D;

            pidController.setTargetValue(target);
            currentPos = motor.getCurrentPosition();
            double power = pidController.update(currentPos);

            motor.setPower(power);

            telemetry.addData("TargetPosition", pidController.targetValue);
            telemetry.addData("CurrentPosition", currentPos);
            telemetry.addData("power", power);
            telemetry.update();        }
    }
}
