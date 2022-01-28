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
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    scoring.goToEnd();
                }
            };
            scoring.tuckPos();
            lift.raise();
            lift.retracting(false);
            delay.delay(run,150);
            movementState = "EXTEND";
        }
    }


    public void lower(){
        scoring.goToStart();
        scoring.tuckPos();
        Runnable run = new Runnable() {
            @Override
            public void run() {
                lift.retracting(true);
                lift.lower();
                scoring.depositReset();
            }
        };

        delay.delay(run,400);

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
            scoring.goToEnd();
            scoring.dump();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    lower();
                }
            };
            delay.delay(run,700);
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
