package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmController{
    public static double P = 0.01, I = 0.001, D = 0.04;
    public static int target = 0;
    public int currentPos;
    public PIDController pidController;

    int MaxPoz = 800;
    int MidPoz = 500;
    int MinPoz = 0;
    DcMotorEx armMotor;
    public ArmController(HardwareMap map)
    {
        pidController = new PIDController(P, I, D);
        pidController.maxOutput = 0.8;
        armMotor = map.get(DcMotorEx.class, "m2e");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goUp()
    {
        target = MaxPoz;
        pidController.targetValue = target;
    }
    public void goDown()
    {
        target = MinPoz;
        pidController.targetValue = target;
    }
    public void goMid()
    {
        target = MidPoz;
        pidController.targetValue = target;
    }
    public void update()
    {
        currentPos = armMotor.getCurrentPosition();
        double power = pidController.update(currentPos);
        armMotor.setPower(power);
    }
}
