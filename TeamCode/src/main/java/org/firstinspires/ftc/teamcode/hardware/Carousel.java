package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Carousel extends Mechanism {
    private DcMotor carousel;
    public static Double speed = 0.5;
    public static Double phase0Speed = 0.5;
    public static Double phase1Speed = 1.0;
    private DelayCommand delay = new DelayCommand();

    private MotionProfile b = MotionProfileGenerator.generateSimpleMotionProfile(
            new MotionState(0, 0, 0),
            new MotionState(60, 1, 0),
            1,
            40
    );

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
        delay.delay(phase1, 700);
        delay.delay(phase2,800);

    }
    public void rrrun(MotionProfile profile, ElapsedTime timer, int direction) {
        carousel.setPower(direction*profile.get(timer.seconds()).getV());
    }

}