package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel extends Mechanism {
    private DcMotor carousel;
    public static Double speed = 0.5;

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


}