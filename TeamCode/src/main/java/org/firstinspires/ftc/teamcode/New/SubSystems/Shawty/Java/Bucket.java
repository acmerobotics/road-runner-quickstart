package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    Servo Bucket1;
    Servo Bucket2;
    State state;

    public Bucket(HardwareMap hardwareMap){
        Bucket1 = hardwareMap.get(Servo.class, "Bucket");
        Bucket2 = hardwareMap.get(Servo.class, "Bucket");
    }

    public void update(){
        Bucket1.setPosition(state.pos);
        Bucket2.setPosition(-state.pos);
    }

    enum State{
        STATIONARY(0),
        DROPPING(1);

        final double pos;
        State(double pos) {
            this.pos = pos;
        }
    }
}
