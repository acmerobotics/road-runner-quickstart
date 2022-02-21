package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftScoringV2 extends Mechanism{
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();
    private DelayCommand delay = new DelayCommand();
    private String movementState; //STILL, EXTEND, RETRACT
    private String goalReach;
    private boolean formerExtend = false;
    public static int testingInt = 300;

    public void init(HardwareMap hwmap){
        lift.init(hwmap);
        scoring.init(hwmap);
        movementState = "DETRACT";
    }

    public void raise(String goal){
        //FOR MIDDLE GOAL GO 9
        switch (goal) {
            case "highgoal": {
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        scoring.goToLowGoal();
                    }
                };
                scoring.tuckPos();
                lift.raiseHigh();
                //lift.retracting(false);
                delay.delay(run, 150);
                movementState = "EXTEND";
                break;
            }
            case "midgoal": {
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        scoring.goToLowGoal();
                    }
                };

                scoring.tuckPos();

                //CHANGE THIS LINE TO WHATEVER HEIGHT YOU NEED
                lift.raiseMid();
                delay.delay(run, 150);
                movementState = "EXTEND";
                break;
            }
            case "lowgoal": {
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        scoring.goToLowGoal();
                    }
                };
                scoring.tuckPos();
                delay.delay(run, 100);
                movementState = "EXTEND";
                break;
            }
        }
    }


    public void lower(String goal){
        if(!goal.equals("lowgoal")) {
            scoring.goToStart();
            scoring.tuckPos();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    //lift.retracting(true);
                    lift.lower();
                    scoring.depositReset();

                }
            };
            if(goal.equals("highgoal")) delay.delay(run, 700);
            else if(goal.equals("midgoal")) delay.delay(run, 1100);

        }
        else{
            scoring.goToStart();
            scoring.lowGoalTuck();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    //lift.retracting(true);
                    scoring.depositReset();

                }
            };

            delay.delay(run, 700);
        }
        movementState = "DETRACT";
    }

    public void toggle(String goal){
        if (movementState.equals("DETRACT")){
            raise(goal);
        }

        else{
            lower(goal);
        }
        goalReach = goal;
    }
    public void release(){
        if(!movementState.equals("DETRACT")){
            scoring.dump();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };
            delay.delay(run,300);
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

        lift.retracting(movementState.equals("DETRACT"));

        lift.update();

    }

    public double getPos() {
        return lift.getCurrentPosition();
    }

    public double getTargetPos() {
        return lift.getTargetPosition();
    }

}
