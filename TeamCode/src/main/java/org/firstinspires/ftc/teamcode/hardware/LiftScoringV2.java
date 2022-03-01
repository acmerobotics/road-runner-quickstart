package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;

public class LiftScoringV2 extends Mechanism{
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor freightSensor = new FreightSensor();
    private String movementState; //EXTEND, RETRACT
    private String goalReach;
    private boolean formerExtend = false;
    public static int testingInt = 300;

    public void init(HardwareMap hwmap){
        lift.init(hwmap);
        scoring.init(hwmap);
        freightSensor.init(hwmap);
        movementState = "DETRACT";
    }
    public void init(HardwareMap hwmap, FreightSensor freightSensor){
        init(hwmap);
        setFreightSensor(freightSensor);
    }
    public void setFreightSensor(FreightSensor freightSensor){
        this.freightSensor = freightSensor;
    }

    /**
     * raises slide to desired position
     * @param goal goal state desired ("highgoal", "midgoal", "lowgoal")
     */
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
                delay.delay(run, 0);
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
                delay.delay(run, 0);
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
                delay.delay(run, 0);
                movementState = "EXTEND";
                break;
            }

            case "cap": {
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        scoring.goToEnd();
                    }
                };
                scoring.tuckPos();
                lift.raiseHigh();
                //lift.retracting(false);
                delay.delay(run, 0);
                movementState = "EXTEND";
                break;

            }
        }
    }

    @Deprecated
    public void readyCap(){
        Runnable run = new Runnable() {
            @Override
            public void run() {
                scoring.pickUpCap();
            }
        };
        scoring.tuckPos();
        scoring.goToEnd();
        delay.delay(run, 500);
        movementState = "EXTEND";
    }

    @Deprecated
    public void raiseCap(){
        lift.raiseHigh();
        scoring.reposCap();
    }


    public void bottom() {
        movementState = "DETRACT";
        lift.lower();
    }


    /**
     * lowers slides depending on goal state
     * @param goal goal state desired ("highgoal", "midgoal", "lowgoal")
     */
    public void lower(String goal){
        //auton vs teleop changes?
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

            if(goal.equals("highgoal")) delay.delay(run, 0);
            else if(goal.equals("cap")) delay.delay(run, 0);

            else if(goal.equals("midgoal")) delay.delay(run, 0);

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

            delay.delay(run, 100);
        }
        movementState = "DETRACT";
    }

    /**
     *
     * @param goal toggles based on goal state desired ("highgoal", "midgoal", "lowgoal")
     */
    public void toggle(String goal){
        if (movementState.equals("DETRACT")){
            raise(goal);
        }

        else{
            lower(goal);
        }
        goalReach = goal;
    }

    /**
     * dumps with deposit and lowers slides in tandem
     */
    public void releaseHard(){
        if(!movementState.equals("DETRACT")){
            scoring.dumpHard();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };
            delay.delay(run,300);
        }
    }

    public void releaseSoft(){
        if(!movementState.equals("DETRACT")){
            scoring.dumpSoft();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };
            delay.delay(run,300);
        }
    }

    /**
     *
     * @return movement state of slides returned ("highgoal", "midgoal", "lowgoal")
     */
    public String getMovementState(){
        return movementState;
    }

    /**
     * update values (Ex. lift PID)
     */
    public void update(){
//        boolean stillCondition = Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) < 0.9;
//        if(stillCondition) {
//            movementState = "STILL";
//            lift.retracting(false);
//        }

        if(freightSensor.hasFreight() && movementState.equals("DETRACT")){
            scoring.tuckPos();
        }
        else if (movementState.equals("DETRACT")){
            scoring.depositReset();
        }
        lift.retracting(movementState.equals("DETRACT"));

        lift.update();

    }

    /**
     *
     * @return encoder based position of lift
     */
    public double getPos() {
        return lift.getCurrentPosition();
    }

    /**
     *
     * @return target position of lift
     */
    public double getTargetPos() {
        return lift.getTargetPosition();
    }

}
