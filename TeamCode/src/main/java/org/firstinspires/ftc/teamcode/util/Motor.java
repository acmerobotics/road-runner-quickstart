package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Motor extends Component{
    private boolean reverse;
    private float speed;
    public DcMotor motor;

    public Motor(int port, String name, HardwareMap map, boolean reverse){
        super(port, name);
        this.reverse = reverse;
        this.speed = 0;
        motor = map.dcMotor.get(name);

        if(reverse){
            motor.setDirection(DcMotor.Direction.REVERSE);
        }

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setSpeed(float speed) {
        this.speed = speed;
        motor.setPower(speed);
    }

    public float getEncoderValue(){
        return motor.getCurrentPosition();

    }

    public void setTarget(int ticks){

        motor.setTargetPosition(ticks);

    }

    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}