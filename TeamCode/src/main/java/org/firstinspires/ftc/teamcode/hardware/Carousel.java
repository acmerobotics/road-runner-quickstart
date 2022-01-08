package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Carousel extends Mechanism {
    private DcMotor carousel;
    private DcMotorSimple.Direction properDirection = DcMotorSimple.Direction.FORWARD;
    private static Double speed = 1.0;

    //Goal in init is to just initialize the motors
    public void init(HardwareMap hwMap) {
        carousel = hwMap.dcMotor.get("carousel");
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Simple intake and outake code, directing the motors to power and rotate in a certain direction.

    public boolean reverse(){
        if (carousel.getDirection() == DcMotorSimple.Direction.FORWARD)
            carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        else if (carousel.getDirection() == DcMotorSimple.Direction.REVERSE)
            carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        return carousel.getDirection() == properDirection;
    }

    public boolean forwards() {return carousel.getDirection() == properDirection;}


    public void run(boolean run){
        if(run) carousel.setPower(speed);
        else carousel.setPower(0);
    }


}