package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class MotorEx extends Component{
    private boolean reverse;
    private float speed;
    public DcMotorEx motor;

    public MotorEx(int port, String name, HardwareMap map, boolean reverse){
        super(port, name);
        this.reverse = reverse;
        this.speed = 0;
        motor = map.get(DcMotorEx.class, name);

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

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}