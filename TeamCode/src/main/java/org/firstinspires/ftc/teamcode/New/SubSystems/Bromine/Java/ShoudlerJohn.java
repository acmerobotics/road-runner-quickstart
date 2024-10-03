package org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.Java;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDFcontroller;
import org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Kotlin.PIDParams;

public class ShoudlerJohn {
    DcMotor Shoulder;
    State state;
    PIDParams shoulderParams = new PIDParams(0,0,0,0);
    PIDFcontroller shoulderPID = new PIDFcontroller(shoulderParams);
    int target;

    public ShoudlerJohn(HardwareMap hardwareMap){
        Shoulder = hardwareMap.get(DcMotor.class,"Shoulder");
    }

    public void update(){
        target = state.encoderPos;
        double power = shoulderPID.calculate(target - Shoulder.getCurrentPosition());

        if(state == State.IDLE){
            power = 0;
        }

        Shoulder.setPower(power);
    }

    enum State{
        BASKET(0),
        CLIP(0),
        SUBMERSIBLE(0),
        GROUND(0),
        STATIONARY(0),
        IDLE(0);

        public final int encoderPos;
        State(int encoderPos) {
            this.encoderPos = encoderPos;
        }
    }
}
