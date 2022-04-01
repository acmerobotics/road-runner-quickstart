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
    public static double LIMIT_L_START = 1;
    public static double LIMIT_L_END = 0;

    public static double LIMIT_R_START = 0.03;
    public static double LIMIT_R_END = 1;

    ////////////DEPO SERVO LIMITS
    public static double LIMIT_DEPO_START = 0.0;
    public static double LIMIT_DEPO_END = 1.0;

    private GearedServos pivotArm = new GearedServos(
            "armServoR", LIMIT_R_START, LIMIT_R_END,
            "armServoL", LIMIT_L_START, LIMIT_L_END
            );

    private ServoManager kicker = new ServoManager("deposit",LIMIT_DEPO_START,LIMIT_DEPO_END);

    /////ARM SERVO POSITIONS
    //Constants are fucked and thrown around
    public static double armStartPos = 0.07; //homed position
    public static double armEndPos = 0.7; //goes to... end?
    public static double armLowGoalPos = 0.7; //goes to... far end for low goal?
    public static double armMidPos = 0.7; //goes to... the middle that's not the middle?
    public static double armReadyPosAuton = 0.1; //.35 old
    public static double armReadyPosTeleop = 0.35; //.35 old

    public static double armDuckPos = 0.8;
    /////DEPO SERVO POSITIONS
    public static double depoStartPos = 0.8; //init position of depo
    public static double depoEndPos = 1.0; //far end position of depo
    public static double depoTuckPos= 1.0; //tuck position for movement while going upwards
    public static double depoDumpPos_Hard = 0.2; //position to go to for dump movement HARD
    public static double depoDumpPos_Soft = 0.5; //position to go to for dump movement SOFT

    public static double depoLowGoalPos = depoStartPos; //position to go to for lowGoal prep
    public static double depoCapStonePos = 0.9; //position to go to for capStone prep

    public boolean auton = false;
    private boolean formerBoolArm;
    private boolean formerBoolDeposit;
    private boolean homed;

    @Override
    public void init(HardwareMap hwMap) {
        pivotArm.init(hwMap);
        kicker.init(hwMap);
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
        //kicker.setPosRatio(depoLowGoalPos);
        homed = false;
        
    }


    /**
     * repositioning for picking up cap
     */
    @Deprecated
    public void pickUpCap(){
        pivotArm.goTo(armLowGoalPos);
        kicker.setPosRatio(0.9);
    }

    /**
     * repositioning of arm and depo for placing cap
     */
    @Deprecated
    public void reposCap() {
        pivotArm.goTo(armMidPos);
        kicker.setPosRatio(1);
    }

    /**
     * tucks for lowgoal scoring
     */
    public void lowGoalTuck(){
        kicker.setPosRatio(depoLowGoalPos);
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
        return kicker.getPosRatio();
    }

    /**
     * sets depo to tuck pos
     */
    public void tuckPos(){
        kicker.setPosRatio(depoTuckPos);
    }

    /**
     * dumps freight
     */
    public void dumpHard() {
        kicker.setPosRatio(depoDumpPos_Hard);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(depoStartPos);
            }

        };
        //delay.delay(run, 350);

    }

    public void dumpSoft() {
        kicker.setPosRatio(depoDumpPos_Soft);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(depoStartPos);
            }

        };
        //delay.delay(run, 350);

    }

    public void dumpDuck(){
        kicker.setPosRatio(depoDumpPos_Soft);
        pivotArm.goTo(armDuckPos);
        Runnable run = new Runnable() {
            @Override
            public void run() {
                kicker.setPosRatio(depoStartPos);
            }

        };
    }

    /**
     * resets depo position
     */
    public void depositReset() {
        kicker.setPosRatio(depoStartPos);
    }

    public void readyPos(){
        if(auton) pivotArm.goTo(armReadyPosAuton);
        else{
            pivotArm.goTo(armReadyPosTeleop);
        }
    }



    /** goes to tucked position
     *
     */
    @Deprecated
    public void tuck(){
        pivotArm.goTo(armMidPos);
        kicker.setPosRatio(depoTuckPos);
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
