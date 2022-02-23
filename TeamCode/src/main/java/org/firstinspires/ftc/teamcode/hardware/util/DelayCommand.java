package org.firstinspires.ftc.teamcode.hardware.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import static java.util.concurrent.TimeUnit.SECONDS;

public class DelayCommand {

    //TODO:
    // To run a method as a runnable event, do this:
    // Runnable targetMethod = new Runnable() {
    // public void run() {
    //    myObject.objectMethod();
    // }};
    // ~~As a novice programmer, I'm not sure about what this really does and how safe it is.~~
    // Currently it is unstable, but this can be fixed by making the event cancellable depending
    // on the state of the opMode (if stop is requested)
    // Delay is in milliseconds. TimeUnit can be changed.
    // In event, input your method converted to a Runnable or utilize a lambda. Note that a lambda
    // CANNOT change values of a variable and a Runnable needs to be utilized in that event.
    ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();



    public void delay(Runnable event, int delay){
        executorService.schedule(event, delay, TimeUnit.MILLISECONDS);
    }

    public void loop(Runnable event, int rate, LinearOpMode opMode){

        final ScheduledFuture<?> beeperHandle =
                executorService.scheduleAtFixedRate(event, 0, rate, TimeUnit.MILLISECONDS);
        Runnable end = new Runnable() {
            @Override
            public void run() {
                if(!opMode.opModeIsActive()){
                    beeperHandle.cancel(true);
                }
            }
        };
        final ScheduledFuture<?> cancelhandle =
                executorService.scheduleAtFixedRate(event, 0, rate, TimeUnit.MILLISECONDS);
    }
}