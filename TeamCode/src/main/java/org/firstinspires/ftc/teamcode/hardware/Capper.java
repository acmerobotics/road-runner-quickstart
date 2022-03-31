package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Capper extends ServoMechanism{
    //capper servo pos
    public static double GRAB = .15;
    public static double CAPPING_UP = 0.35;
    public static double CAPPING_DOWN = 0.15;
    public static double IDLE = 1;
    public static double CAP_POS = 5.5;


    Servo capper;
    @Override
    public void init(HardwareMap hwMap) {
        capper = hwMap.get(Servo.class, "capper");
        reset();
    }
    /* logic
        //CONTROLS GOING UP OR GOING DOWN
        if (button1){
            capper.raise();
            lift.setPos(CAPPING_POS);
        }
        else {
            capper.reset()
            lift.reset();
        }
        //CONTROLS CAPPING IF RAISED, ELSE TOGGLES GRABBING
        if(button2){
            if(lift.state = "CAPPING") {
                if (former){
                    capper.raise();
                } else {
                    capper.release();
                }
            } else {
                if (former){
                    capper.grab();
                } else {
                    capper.reset();
                }
            }
            
        }
    */
    public void reset() {
        capper.setPosition(IDLE);
    }

    public void grabCap() {
        capper.setPosition(GRAB);
    }

    public void raise() {
        capper.setPosition(CAPPING_UP);
    }
    public void release(){
        capper.setPosition(CAPPING_DOWN);
        
    }

    @Override
    public void run(boolean bool) {

    }
}
