package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftScoringV2 extends Mechanism{
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();
    private DelayCommand delay = new DelayCommand();
    private String movementState; //STILL, EXTEND, RETRACT
    private boolean formerExtend = false;
    public static int testingInt = 300;

    public void init(HardwareMap hwmap){
        lift.init(hwmap);
        scoring.init(hwmap);
        movementState = "DETRACT";
    }

    public void raise(String goal){
        if(goal.equals("highgoal")){
            Runnable extendOut = new Runnable() {
                @Override
                public void run() {
                    scoring.goToEnd();
                }
            };
            lift.setTargetPosition(7);
            delay.delay(extendOut,100);
            movementState = "EXTEND";
        }
    }


    public void lower(){
        Runnable detract = new Runnable() {
            @Override
            public void run() {
                lift.setTargetPosition(0);
                lift.retracting(true);
            }
        };

        scoring.goToStart();
        delay.delay(detract,testingInt);
        movementState = "DETRACT";
    }

    public void toggle(String goal){
        if (movementState.equals("DETRACT")){
            raise(goal);
        }

        else{
            lower();
        }
    }
    public void release(){
        if(!movementState.equals("DETRACT")){
            scoring.dump();
            Runnable toggle = new Runnable() {
                @Override
                public void run() {
                    toggle("highgoal");
                }
            };

            delay.delay(toggle,350);
        }
    }

    public String getMovementState(){
        return movementState;
    }

    public void update(){
//        boolean stillCondition = Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) < 0.9;
//        if(stillCondition) {
//            movementState = "STILL";
//            lift.retracting(false);
//        }

        if(movementState.equals("DETRACT")) lift.retracting(true);
        else lift.retracting(false);

        lift.update();

    }


}
