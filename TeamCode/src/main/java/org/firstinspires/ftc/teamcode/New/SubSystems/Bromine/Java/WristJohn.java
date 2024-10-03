package org.firstinspires.ftc.teamcode.New.SubSystems.Bromine.Java;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristJohn {
    Servo Wrist;
    State state = State.STATIONARY;

    public WristJohn(HardwareMap hardwareMap){
        Wrist = hardwareMap.get(Servo.class, "Wrist");
    }

    enum State{
        BASKET(0),
        CLIP(0),
        SUBMERSIBLE(0),
        GROUND(0),
        STATIONARY(0);

        public final double servoPos;
        State(double servoPos) {
            this.servoPos = servoPos;
        }
    }

    public void update(){
        switch (state){
            case BASKET: Wrist.setPosition(State.BASKET.servoPos); break;
            case CLIP: Wrist.setPosition(State.CLIP.servoPos); break;
            case SUBMERSIBLE: Wrist.setPosition(State.SUBMERSIBLE.servoPos); break;
            case GROUND: Wrist.setPosition(State.GROUND.servoPos); break;
            case STATIONARY: Wrist.setPosition(State.STATIONARY.servoPos); break;
        }
    }


}
