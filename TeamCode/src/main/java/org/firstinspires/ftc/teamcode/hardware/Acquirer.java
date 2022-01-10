package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Code for acquirer mech
public class Acquirer extends Mechanism {
    private DcMotor acquirerM;
    public static double max_pow = 0.8;
    public static double outake = 1.0 * max_pow;
    public static double intake = 1.0 * max_pow;
    //Goal in init is to just initialize the motors
    public void init(HardwareMap hwMap) {
        acquirerM = hwMap.dcMotor.get("intake");
        acquirerM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Simple intake and outake code, directing the motors to power and rotate in a certain direction.
    public void intake(double power){
        acquirerM.setPower(power);
    }

    public void outake(double power){
        acquirerM.setPower(-power);
    }

    public void run(boolean left, boolean right){
        if(left) outake(outake);
        else if (right) intake(intake);
        else intake(0);
    }

}