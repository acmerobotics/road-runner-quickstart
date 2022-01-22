package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
public class ScoringArm extends ServoMechanism{
    DelayCommand delay = new DelayCommand();

    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     * If it works, you're good to go
     ****/

    /////////////ARM SERVO LIMITS
    public static double LIMIT_L_START = 0.1;
    public static double LIMIT_L_END = 0.8;

    public static double LIMIT_R_START = 0.98;
    public static double LIMIT_R_END = 0.3;

    ////////////DEPO SERVO LIMITS
    public static double LIMIT_DEPO_START = 0.1;
    public static double LIMIT_DEPO_END = 0.8;

    private GearedServos pivotArm = new GearedServos(
            "armServoR", LIMIT_R_START, LIMIT_R_END,
            "armServoL", LIMIT_L_START, LIMIT_L_END
            );

    private Arm deposit = new Arm("deposit",LIMIT_DEPO_START,LIMIT_DEPO_END);

    /////ARM SERVO POSITIONS
    public static double armStartPos = 0.04;
    public static double armEndPos = 0.9;
    public static double armMidPos = 0.6;

    /////DEPO SERVO POSITIONS
    public static double depoStartPos = 0.45;
    public static double depoEndPos = 0.9;
    public static double depoTuckPos= 0.1;

    private boolean formerBoolArm;
    private boolean formerBoolDeposit;
    private boolean homed;
    @Override
    public void init(HardwareMap hwMap) {
        pivotArm.init(hwMap);
        deposit.init(hwMap);
        goToStart();
        depositReset();
        homed = true;
    }
    //GO TO POS RATIO
    public void goTo(double desiredPosition){
        pivotArm.goTo(desiredPosition);
        homed = false;
    }
    // MAX

    public void goToLowGoal(){
        goTo(armEndPos);
        homed = false;
        
    }
    public void goToEnd(){
        pivotArm.goTo(armMidPos);
        homed = false;
    }
    //RESET
    public void goToStart(){
        pivotArm.goTo(armStartPos);
        homed = true;
    }

    public double getPosPivotArm(){
        return pivotArm.getPos();
    }

    public double getPosDeposit(){
        return deposit.getPosRatio();
    }

    public void tuckPos(){
        deposit.setPosRatio(depoTuckPos);
    }

    public void tuck(){
        pivotArm.goTo(armMidPos);
        deposit.setPosRatio(depoTuckPos);
        homed = false;
    }

    public void dump() {
        deposit.setPosRatio(depoEndPos);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                deposit.setPosRatio(depoTuckPos);
            }

        };
        delay.delay(run, 350);

    }
    public void depositReset() {
        deposit.setPosRatio(depoStartPos);
    }
    public void run(boolean bool){
        if(bool){
            formerBoolArm = true;
        }

        if(formerBoolArm){
            if(!bool){
                if(homed) goToEnd();
                else goToStart();
                formerBoolArm = false;
            }
        }
    }

    public boolean homed(){return homed;}

}
