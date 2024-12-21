package org.firstinspires.ftc.teamcode.hardware.limp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "ShoulderLIMP", group = "LIMP")
// LIMP - Lacking Instructions Mode (for) Precision
public class ShoulderLIMP extends OpMode {

    private final double ticks_in_degree =  3.0 / 23.0;

    private DcMotorEx shoulder_right;
    private DcMotorEx shoulder_left;
    private DcMotorEx viper;

    @Override
    public void init() {

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
        shoulder_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoulder_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        int armPos = shoulder_left.getCurrentPosition();
        telemetry.addData("pos", armPos);
        telemetry.addData("to degrees", (armPos * ticks_in_degree) - 42);
    }
}
