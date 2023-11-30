package org.firstinspires.ftc.teamcode.testing.structureOptions.isolatedNavigationClass;

public class Navigator {
    private Movement pos;

    private Hardware hardware;



    private void sleep(long milliseconds){
        try{
            Thread.sleep(milliseconds);
        }catch(InterruptedException e){
            // I have no clue what this does.
            Thread.currentThread().interrupt();
        }
    }

    void forward(double power, long ms){
        MecDT dt = hardware.getDT();
        dt.mecPow(power, 0, 0);
        sleep(ms);
        dt.stop();
    }

    void setHardware(Hardware hardware){
        this.hardware = hardware;
    }
}
