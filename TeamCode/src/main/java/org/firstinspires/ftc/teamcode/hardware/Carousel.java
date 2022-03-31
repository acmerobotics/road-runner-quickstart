package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;


@Config
public class Carousel extends Mechanism {
    private DcMotor carousel;
    ElapsedTime time = new ElapsedTime();
    public static Double speed = 0.35;

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

    public static double maxV = 1;
    public static double maxA = 0.1;
    public static double startV = 0.5;
    public static double startA = 0;

    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, startV, startA),
            new MotionState(60, maxV, 0),
            maxV,
            maxA
    );

    // single r run is first iteration; autorun is second; triple rrrun is third
    public void init(HardwareMap hwMap) {
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * input left and right boolean for which carousel runs. if both are true, defaults to left
     * constant power motion profile
     * @param left determines if left carousel runs
     * @param right determines if right carousel runs
     */
    public void run(boolean left, boolean right){
        if(left) carousel.setPower(speed);
        else if(right) carousel.setPower(-speed);
        else carousel.setPower(0);
    }
    public void runmax(boolean left, boolean right){
        if(left) carousel.setPower(1);
        else if(right) carousel.setPower(-1);
        else carousel.setPower(0);
    }
    /**
     * runs the carousel at the static double speed
     * constant power motion profile
     * @param run if true, runs carousel
     */
    public void run(boolean run){
        if (run) carousel.setPower(speed);
        else carousel.setPower(0);
    }

    /**
     *     utilizes asynchronous speed changes
     *     jerk motion profile
     */
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

    /**
     * utilizes roadrunner motion profiling to set carousel speed over a duration accordingly to desired MotionProfile
     * Theoretically the best option
     * @param profile motion profile to utilize for roadrunner MP
     * @param timer timer to utilize for roadrunner MP
     * @param direction direction of carousel; -1 reverses power (counter clockwise)
     */
    public void rrrun(MotionProfile profile, ElapsedTime timer, int direction) {
        carousel.setPower(direction*profile.get(timer.seconds()).getV());
    }

    public void rrrun(ElapsedTime timer, int direction){
        carousel.setPower(direction * profile.get(timer.seconds()).getV());
    }

}