package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Capper extends ServoMechanism{
    //capper servo pos
    public static double grab = .125;
    public static double capping = 1;
    public static double idle = 0;
    Servo capper;
    @Override
    public void init(HardwareMap hwMap) {
        capper = hwMap.get(Servo.class, "capper");
    }

    public void idle() {
        capper.setPosition(idle);
    }

    public void grabCap() {
        capper.setPosition(grab);
    }

    public void cap() {
        capper.setPosition(capping);
    }
    @Override
    public void run(boolean bool) {

    }
}
