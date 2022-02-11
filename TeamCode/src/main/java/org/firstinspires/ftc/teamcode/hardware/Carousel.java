package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
public class Carousel extends Mechanism {
    private DcMotor carousel;
    ElapsedTime time = new ElapsedTime();
    public static Double speed = 0.25;

    public static Double phase0Speed = 0.55;
    public static Double phase1Speed = 1.0;

    public static int phase0Timer = 1000;
    public static int phase1Timer = 1100;

    public static double coastingValue = 0.0;
    public static double coastingDefinition = 0.0;

    public static double timeRun = 1.0;
    public static double rate = 0.1;
    public static double refreshRate = 0.1;
    public static double currentPower = 0;

    private DelayCommand delay = new DelayCommand();

    public void init(HardwareMap hwMap) {
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(boolean left, boolean right){
        if(left) carousel.setPower(speed);
        else if(right) carousel.setPower(-speed);
        else carousel.setPower(0);
    }

    public void run(boolean run){
        if (run) carousel.setPower(speed);
        else carousel.setPower(0);
    }

    public void autoRun(int direction) {

        Runnable phase0 = new Runnable() {
            @Override
            public void run() {
                carousel.setPower(direction * phase0Speed);
            }
        };

        Runnable phase1 = new Runnable() {
            @Override
            public void run() {
                carousel.setPower(direction * phase1Speed);
            }
        };

        Runnable phase2 = new Runnable() {
            @Override
            public void run() {
                carousel.setPower(0);
            }
        };

        delay.delay(phase0,0);
        delay.delay(phase1, phase0Timer);
        delay.delay(phase2,phase1Timer);

    }

    public void run(){
        if(time.time(TimeUnit.SECONDS) >= timeRun){
            carousel.setPower(0);
        }
        else{
            if(time.time(TimeUnit.SECONDS) % 0.1 == 0){

            }
        }
    }

}