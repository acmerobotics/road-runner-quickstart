package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    DcMotor ArmMotor;

    public Arm(DcMotor theArm){
        ArmMotor = theArm;
    }
    public Arm(HardwareMap hardwareMap){
        ArmMotor = hardwareMap.get(DcMotor.class, Constants.Arm2ConfigName);
    }
    public void ArmUp(double power){
        ArmMotor.setTargetPosition(Constants.ArmUpTicks);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
    }

    public void ArmUpAuto(double power){
        ArmMotor.setTargetPosition(Constants.ArmUpTicksAuto);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
    }

    public void ArmDown(double power){
        double multiplier = 1;
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power*multiplier);
//        while(ArmMotor.isBusy()){
//            ArmMotor.setPower(power*multiplier);
//            if(ArmMotor.getCurrentPosition() < 950){
//                multiplier = 0.1;
//            }
//        }
    }

    public void ArmChangePos(int change, double power){
        ArmMotor.setTargetPosition(this.getPosition() + change);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setPower(power);
    }

    public int getPosition(){
        return ArmMotor.getCurrentPosition();
    }

    public void resetPos(){
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
