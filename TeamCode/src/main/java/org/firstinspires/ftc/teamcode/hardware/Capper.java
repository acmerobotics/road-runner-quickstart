package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capper extends ServoMechanism{
    //capper servo pos
    public static double GRAB = .15;
    public static double CAPPING_UP = 0.5;
    public static double CAPPING_DOWN = 0.4;
    public static double IDLE = 0;
    Servo capper;
    @Override
    public void init(HardwareMap hwMap) {
        capper = hwMap.get(Servo.class, "capper");
    }

    public void reset() {
        //lift.reset();
        capper.setPosition(IDLE);
    }

    public void grabCap() {
        capper.setPosition(GRAB);
    }

    public void raise() {
        capper.setPosition(CAPPING_UP);
        //lift.setPos(7.5);
    }
    public void release(){
        capper.setPosition(CAPPING_DOWN);
        
    }
    @Override
    public void run(boolean bool) {

    }
}
