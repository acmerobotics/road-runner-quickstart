package org.firstinspires.ftc.teamcode.hardware;

public class BooleanManager {
    Runnable task;
    boolean formerBoolean = false;
    public BooleanManager(Runnable run){
        //runs the run command
        task = run;
    }

    public void update(boolean checkValue){
        if (checkValue) formerBoolean = true;
        if(formerBoolean && !checkValue){
            formerBoolean = false;

            task.run();
        }
    }

}
