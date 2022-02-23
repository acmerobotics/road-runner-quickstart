package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;

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

    private ServoManager deposit = new ServoManager("deposit",LIMIT_DEPO_START,LIMIT_DEPO_END);

    /////ARM SERVO POSITIONS
    //Constants are fucked and thrown around
    public static double armStartPos = 0.04; //homed position
    public static double armEndPos = 0.7; //goes to... end?
    public static double armLowGoalPos = 1.0; //goes to... far end for low goal?
    public static double armMidPos = 0.6; //goes to... the middle that's not the middle?

    /////DEPO SERVO POSITIONS
    public static double depoStartPos = 0.45; //init position of depo
    public static double depoEndPos = 0.9; //far end position of depo
    public static double depoTuckPos= 0.3; //tuck position for movement while going upwards
    public static double depoDumpPos = 0.5; //position to go to for dump movement
    public static double depoLowGoalPos = 0; //position to go to for lowGoal prep
    public static double depoCapStonePos = 0.9; //position to go to for capStone prep


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

    /**
     * moves to low goal scoring position with arm and depo
     */
    public void goToLowGoal(){

        pivotArm.goTo(armLowGoalPos);
        deposit.setPosRatio(depoLowGoalPos);
        homed = false;
        
    }

    /**
     * repositioning for picking up cap
     */
    @Deprecated
    public void pickUpCap(){
        pivotArm.goTo(armLowGoalPos);
        deposit.setPosRatio(0.9);
    }

    /**
     * repositioning of arm and depo for placing cap
     */
    @Deprecated
    public void reposCap() {
        pivotArm.goTo(armMidPos);
        deposit.setPosRatio(1);
    }

    /**
     * tucks for lowgoal scoring
     */
    public void lowGoalTuck(){
        deposit.setPosRatio(depoLowGoalPos);
    }

    /**
     * moves arm to end pos
     */
    public void goToEnd(){
        pivotArm.goTo(armEndPos);
        homed = false;
    }

    /**
     * moves arm to start pos
     */
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

    /**
     * sets depo to tuck pos
     */
    public void tuckPos(){
        deposit.setPosRatio(depoTuckPos);
    }

    /**
     * dumps freight
     */
    public void dump() {
        deposit.setPosRatio(depoDumpPos);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                deposit.setPosRatio(depoTuckPos);
            }

        };
        //delay.delay(run, 350);

    }

    /**
     * resets depo position
     */
    public void depositReset() {
        deposit.setPosRatio(depoStartPos);
    }

    /** goes to tucked position
     *
     */
    @Deprecated
    public void tuck(){
        pivotArm.goTo(armMidPos);
        deposit.setPosRatio(depoTuckPos);
        homed = false;
    }

    @Deprecated
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

    @Deprecated
    public boolean homed(){return homed;}

}
