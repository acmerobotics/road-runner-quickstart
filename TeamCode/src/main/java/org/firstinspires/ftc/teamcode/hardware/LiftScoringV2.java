package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;

import java.util.concurrent.TimeUnit;

@Config
public class LiftScoringV2 extends Mechanism{
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();
    private DelayCommand delay = new DelayCommand();
    private ElapsedTime timerBlock = new ElapsedTime();

    //for when the block is there
    public static double triggerTime = 100;
    private double timeMarker = 0;
    private boolean countingBlockHeld = false;
    public static int scoringMechBuffer = 250;
    //for when block isnt there
    private ElapsedTime timerEmpty = new ElapsedTime();
    public static double errorThreshold = 200;
    private boolean countingEmptyHeld = false;

    private FreightSensor freightSensor = new FreightSensor();
    private String movementState; //EXTEND, RETRACT
    public String goalReach = "lowgoal";
    private boolean readyPosition = false;
    private boolean formerExtend = false;
    private boolean formerFreight = false;
    private boolean auton = false;
    private boolean shooting = false;
    public static int testingInt = 300;

    public void init(HardwareMap hwmap){
        timerEmpty.reset();
        timerBlock.reset();
        lift.init(hwmap);
        scoring.init(hwmap);
        movementState = "DETRACT";
    }


    public void init(HardwareMap hwmap, FreightSensor freightSensor){
        init(hwmap);
        setFreightSensor(freightSensor);
        freightSensor.init(hwmap);
    }
    public void setFreightSensor(FreightSensor freightSensor){
        this.freightSensor = freightSensor;
    }

    /**
     * raises slide to desired position
     * @param goal goal state desired ("highgoal", "midgoal", "lowgoal")
     */
    public void raise(String goal){
        readyPosition = false;
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
            case "midgoalTele": {
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        scoring.goToLowGoal();
                    }
                };

                scoring.tuckPos();

                //CHANGE THIS LINE TO WHATEVER HEIGHT YOU NEED
                lift.raiseMidTele();
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
                lift.setTargetPosition(Capper.CAP_POS);
                //lift.retracting(false);
                scoring.tuckPos();
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
        readyPosition = false;
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

            Runnable runHigh = new Runnable() {
                @Override
                public void run() {
                    //lift.retracting(true);
                    Runnable lowerLift = new Runnable() {
                        @Override
                        public void run() {
                            lift.lower();
                        }
                    };
                    scoring.depositReset();
                    delay.delay(lowerLift,scoringMechBuffer);

                }
            };



            if(goal.equals("highgoal")) delay.delay(runHigh, 100);
            else if(goal.equals("midgoal") || goal.equals("midgoalTele")) delay.delay(run, 0);

            else if(goal.equals("cap")){
                delay.delay(run,0);
            }

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
        shooting = true;
        if(!movementState.equals("DETRACT")){
            scoring.dumpHard();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };

            Runnable shootState = new Runnable() {
                @Override
                public void run() {
                    shooting = false;

                }
            };
            delay.delay(shootState,400);

            delay.delay(run,450);
        }
    }

    public void releaseSoft(){
        shooting = true;
        if(!movementState.equals("DETRACT")){
            scoring.dumpSoft();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };

            Runnable shootState = new Runnable() {
                @Override
                public void run() {
                    shooting = false;

                }
            };
            delay.delay(shootState,400);


            delay.delay(run,450);
        }
    }

    public void releaseDuck(){
        if(!movementState.equals("DETRACT")){
            scoring.dumpDuck();
            Runnable run = new Runnable() {
                @Override
                public void run() {
                    toggle(goalReach);
                }
            };
            delay.delay(run,800);
        }
    }

    /**
     *
     * @return movement state of slides returned ("highgoal", "midgoal", "lowgoal")
     */
    public String getMovementState(){
        return movementState;
    }

    public void setReadyPosition(){
        scoring.readyPos();
        readyPosition = true;
    }

    public boolean isReadyPosition(){
        return readyPosition;
    }

    public void setAuton(boolean auton){
        this.auton = auton;
        scoring.auton = auton;
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


        if(true) {
            if(freightSensor.hasFreight() && movementState.equals("DETRACT") && !shooting && !countingBlockHeld) {
                countingBlockHeld = true;
                timerBlock.reset();
            }


            else if(freightSensor.hasFreight() && movementState.equals("DETRACT") && !shooting && countingBlockHeld){
                if(timerBlock.time(TimeUnit.MILLISECONDS) > triggerTime){
                    scoring.tuckPos();
                    setReadyPosition();
                }
            }

            else if(!(freightSensor.hasFreight() && movementState.equals("DETRACT") && !shooting)){
                countingBlockHeld = false;
            }

            else if(readyPosition && !freightSensor.hasFreight() && movementState.equals("DETRACT") && !countingEmptyHeld){
                countingEmptyHeld = true;
                timerEmpty.reset();
            }

            else if(readyPosition && !freightSensor.hasFreight() && movementState.equals("DETRACT") && countingEmptyHeld){
                if(timerEmpty.time(TimeUnit.MILLISECONDS) > errorThreshold){
                    lower("lowgoal");
                }
            }

            else if(readyPosition && freightSensor.hasFreight() && movementState.equals("DETRACT")){
                countingEmptyHeld = false;
            }

//            if (!formerFreight && freightSensor.hasFreight() && movementState.equals("DETRACT") && !readyPosition) {
//                scoring.tuckPos();
//                setReadyPosition();
//
//            } else if (freightSensor.hasFreight() && movementState.equals("DETRACT")) {
//                formerFreight = true;
//            } else if (movementState.equals("DETRACT") && !freightSensor.hasFreight()) {
//                scoring.depositReset();
//                formerFreight = false;
//            }
        }


        lift.retracting(movementState.equals("DETRACT"));

        lift.update();

    }

    public boolean raisingStatus(){
        return freightSensor.hasFreight() && movementState.equals("DETRACT") && !shooting && countingBlockHeld && readyPosition;
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
