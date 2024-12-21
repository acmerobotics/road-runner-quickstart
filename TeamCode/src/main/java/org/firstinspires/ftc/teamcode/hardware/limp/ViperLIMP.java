package org.firstinspires.ftc.teamcode.hardware.limp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "ViperLIMP", group = "LIMP")
// LIMP - Lacking Instructions Mode (for) Precision
public class ViperLIMP extends OpMode {

    private final double ticks_in_inch = 1.0 / 300.0;

    private DcMotorEx viper;

    @Override
    public void init() {

        viper = hardwareMap.get(DcMotorEx.class, "viper_slide");

        viper.setDirection(DcMotorSimple.Direction.FORWARD);

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



    }

    @Override
    public void loop() {
        int vipPos = viper.getCurrentPosition();
        telemetry.addData("pos", vipPos);
        telemetry.addData("to inches", vipPos * ticks_in_inch);
    }
}
