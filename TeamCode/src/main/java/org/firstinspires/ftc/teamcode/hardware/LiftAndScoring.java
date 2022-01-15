package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftAndScoring extends Mechanism{
    private Lift lift = new Lift();
    private ScoringArm scoring = new ScoringArm();

    private boolean formerbBoolToggle = false;
    private boolean formerDepoBool = false;
    private boolean formerLowGoal = false;

    private DelayCommand delay = new DelayCommand();
    public void init(HardwareMap hwmap){
        lift.init(hwmap);
        scoring.init(hwmap);
    }

    public void lowGoal(boolean bool){
        if(bool) formerLowGoal = true;

        if(formerLowGoal){
            if(!bool){
                //do stuff
                scoring.tuckPos();
                scoring.goToLowGoal();
            }
        }
    }

    public void toggleSlides(boolean bool){
        if(bool){
            formerbBoolToggle = true;
        }
        if(formerbBoolToggle){
            if(!bool){
                //dostuff
                if(lift.inAir()){
                    lower();
                }

                else{
                    raise();
                }
                formerbBoolToggle = false;
            }
        }
    }

    public void toggleDepo(boolean bool){
        if(bool){
            formerDepoBool = true;
        }

        if(formerDepoBool){
            if(!bool){
                //dostuff
                scoring.goToEnd();
                scoring.dump();
                formerDepoBool = false;
                Runnable run = new Runnable() {
                    @Override
                    public void run() {
                        lower();
                    }
                };
                delay.delay(run,700);
            }
        }
    }

    public void raise(){
        Runnable run = new Runnable() {
            @Override
            public void run() {
                scoring.goToEnd();            }
        };
        scoring.tuckPos();
        lift.raise();
        lift.retracting(false);
        delay.delay(run,150);

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

    }
    public void update(){
        lift.update();
    }
}
