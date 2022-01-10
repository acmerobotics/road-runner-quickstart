package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends Mechanism {
    private DcMotor leftM;
    private double leftM_max_rotation;
    private double leftM_min_rotation;
    private double rightM_max_rotation;
    private double rightM_min_rotation;
    private DcMotor rightM;
    private final double SPOOL_RADIUS = 1.81;
    public void init(HardwareMap hwMap) {
        leftM = hwMap.dcMotor.get("liftLeft");
        leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightM = hwMap.dcMotor.get("liftRight");
        rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    * extend/retract by rotating x amount
    * if reach certain motor position, stop
    *
    * */
    public void extend(double height){
    }
    public void retract(double height){
    }
    //reset extension/retraction
    public void extend_complete(){

    }
    public void retract_complete(){

    }
}