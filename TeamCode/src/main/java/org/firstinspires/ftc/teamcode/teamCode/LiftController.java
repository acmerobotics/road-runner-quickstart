package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class LiftController{
    public static double P = 0.01, I = 0, D = 0;
    public static int target = 0;
    public int currentPos;
    public PIDController pidController;

    int MaxPoz = 1350;
    int MidPoz = 700;
    int MinPoz = 0;

    DcMotorEx liftMotor;
    public LiftController(HardwareMap map)
    {
        pidController = new PIDController(P, I, D);
        pidController.maxOutput = 0.8;
        liftMotor = map.get(DcMotorEx.class, "m1e");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goUp()
    {
        target = MaxPoz;
    }
    public void goDown()
    {
        target = MinPoz;
    }
    public void goMid()
    {
        target = MidPoz;
    }
    public void goTOPos(int pos)
    {
        target = pos;
    }

    public void up(double x)
    {
        if (target < MaxPoz)
            target = target + (int)((double)x * 10);

    }
    public void down(double x)
    {
        if (target > MinPoz)
            target = target - (int)((double)x * 10);
    }
    public void update()
    {
        pidController.targetValue = target;
        currentPos = liftMotor.getCurrentPosition();
        double power = pidController.update(currentPos);
        liftMotor.setPower(power);
    }
}
