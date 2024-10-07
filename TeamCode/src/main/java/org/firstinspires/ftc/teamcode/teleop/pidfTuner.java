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

    public static double p = 0.001, i = 0, d = 0.0001;
    public static double f = 0.00191399;
    public static double gripPos = 0.54;

    public static int target = 1000;

    private final double ticks_in_degree = 1753 / 180.0;

    private DcMotorEx flip;

    @Override
    public void init() {
        Robot bot = new Robot(hardwareMap);

        controller = new PIDController(p,i,d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flip = (DcMotorEx) bot.flip;

        flip.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flip.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);

        int armPos = flip.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        flip.setPower(power);

        telemetry.addData("armPos", armPos);
        telemetry.addData("target", target);
        telemetry.update();

    }

}

