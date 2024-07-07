package org.firstinspires.ftc.teamcode.teamCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmControllerPID {
    public static double P = 0.04, I = 0.02, D = 2;
    public static int target = 0;
    public int currentPos;
    public PIDController pidController;

    public static int MaxPoz = 900;
    public static int MidPoz = 450;
    public static int MinPoz = 0;
    DcMotorEx armMotor;
    public ArmControllerPID(HardwareMap map)
    {
        pidController = new PIDController(P, I, D);
        pidController.maxOutput = 0.8;
        armMotor = map.get(DcMotorEx.class, "m0e");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
