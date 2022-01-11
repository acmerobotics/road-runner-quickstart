package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringArm extends ServoMechanism{
    DelayCommand delay = new DelayCommand();

    /****
     * WHAT TO DO HERE:
     * Figure out the DESIRED endpoints using ServoTest OpMode
     * Ensure that it's really what you want
     * Test it out, with low commitment
     * If it works, you're good to go
     ****/

    //arm servo positions
    public static double armServoLStart = 0.1;
    public static double armServoLEnd = 0.8;

    public static double armServoRStart = 0.98;
    public static double armServoREnd = 0.3;

    public static double armServoTarget = 5/7.0;

    private GearedServos pivotArm = new GearedServos(
            "armServoR", armServoRStart, armServoREnd,
            "armServoL", armServoLStart, armServoLEnd
            );

    //deposit servo positions
    public static double depositStart = 0.1;
    public static double depositEnd = 0.5;

    public static double depositTarget = 0.5;

    private Arm deposit = new Arm("deposit",depositStart,depositEnd);

    private boolean formerBoolArm;
    private boolean formerBoolDeposit;
    private boolean homed;
    @Override
    public void init(HardwareMap hwMap) {
        pivotArm.init(hwMap);
        deposit.init(hwMap);
        homed = true;
    }
    //GO TO POS RATIO
    public void goTo(double desiredPosition){
        pivotArm.goTo(desiredPosition);
        homed = false;
    }
    // MAX
    public void goToEnd(){
        pivotArm.goToEnd();
        homed = false;
    }
    //RESET
    public void goToStart(){
        pivotArm.goToStart();
        homed = true;
    }

    public double getPosPivotArm(){
        return pivotArm.getPos();
    }

    public double getPosDeposit(){
        return deposit.getPosRatio();
    }



    public void dump(){
        //DUMP ONLY IF HOMED = FALSE

         deposit.setPosRatio(depositTarget);
         Runnable run = new Runnable(){
             @Override
            public void run(){
                 deposit.startPos();
             }

        };

         delay.delay(run, 500);
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

    public void deposit(boolean bool){
        if(bool) formerBoolDeposit = true;

        if(formerBoolDeposit){
            if(!bool){
                if(!homed) dump();
                formerBoolDeposit = false;
            }
        }

    }

}
