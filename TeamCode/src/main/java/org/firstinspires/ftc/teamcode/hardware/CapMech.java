package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class CapMech extends Mechanism{
    enum CapStates {
                PICK,
                IDLE,
                CAP
    };
    CapStates capState = CapStates.IDLE;

    ServoManager capArm = new ServoManager("capMech",0.0,1.0);

    public static double pickPos = 0.0;
    public static double idlePos = 0.5;
    public static double capPos = 1.0;

    public void init(HardwareMap hwmap){
        capArm.init(hwmap);
    }

    public void idlePos(){
        capState = CapStates.IDLE;
        capArm.setPosRatio(idlePos);
    }

    public void pickPos(){
        capState = CapStates.PICK;

        capArm.setPosRatio(pickPos);
    }

    public void capPos(){
        capState = CapStates.CAP;
        capArm.setPosRatio(capPos);
    }

    public CapStates capState(){
        return capState;
    }
}
