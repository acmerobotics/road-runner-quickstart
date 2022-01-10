package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Mechanism {
    private DcMotor leftM;
    private DcMotor rightM;
    public void init(HardwareMap hwMap) {
        leftM = hwMap.dcMotor.get("liftLeft");
//        leftM.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        rightM = hwMap.dcMotor.get("liftRight");
//        rightM.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
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