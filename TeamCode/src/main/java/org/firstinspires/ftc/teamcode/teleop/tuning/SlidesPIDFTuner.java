package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class SlidesPIDFTuner extends OpMode {
    private PIDController controller1;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.006;

    public static int target = 0;
    private final double angleSlides = 60;

    private DcMotorEx slides1;
    private DcMotorEx slides2;

    @Override
    public void init() {
        controller1 = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides1.setDirection(DcMotorSimple.Direction.REVERSE);
        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller1.setPID(p, i, d);
        int slides1Pos = slides1.getCurrentPosition();

        double pid1 = controller1.calculate(slides1Pos, target);
        double ff = Math.cos(Math.toRadians(angleSlides)) * f;

        double power1 = pid1 + ff;

        slides1.setPower(power1);
        slides2.setPower(power1);

        telemetry.addData("pos1 ", slides1Pos);
        telemetry.addData("target ", target);
        telemetry.addData("motor 1 current", slides1.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("motor 2 current", slides2.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}