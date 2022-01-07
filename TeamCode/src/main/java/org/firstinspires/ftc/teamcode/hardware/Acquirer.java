package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Code for acquirer mech
public class Acquirer extends Mechanism {
    private DcMotor acquirerM;

    //Goal in init is to just initialize the motors
    public void init(HardwareMap hwMap) {
        acquirerM = hwMap.dcMotor.get("intake");
        acquirerM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Simple intake and outake code, directing the motors to power and rotate in a certain direction.
    public void intake(float power){
        acquirerM.setPower(power);
    }

    public void outake(float power){
        acquirerM.setPower(-power);
    }

    public void run(float left_trigger, float right_trigger){
        if (left_trigger > 0.3) outake(left_trigger);
        else if (right_trigger > 0.3) intake(right_trigger);
        else{
            intake(0);
        }
    }
}