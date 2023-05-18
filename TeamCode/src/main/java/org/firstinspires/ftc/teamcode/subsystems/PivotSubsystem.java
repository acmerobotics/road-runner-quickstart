package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotSubsystem extends SubsystemBase {

    private final DcMotorEx pivot;
    public double currentVelocity;
    public int currentPosition;

    public PivotSubsystem(HardwareMap hwMap){
        pivot = hwMap.get(DcMotorEx.class, "pivot");
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setVelocity(double velocity){
        pivot.setPower(velocity);
    }

    public double getVelocity(){
        currentVelocity = pivot.getVelocity();
        return currentVelocity;
    }

    public void setPosition(int angle){
        pivot.setTargetPosition(angle);
    }

    public int getPosition(){
        currentPosition = pivot.getCurrentPosition();
        return currentPosition;
    }


}

