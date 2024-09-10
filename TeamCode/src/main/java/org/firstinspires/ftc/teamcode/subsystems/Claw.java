package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends Subsystem{

    public enum ClawStates {
        OPEN (1),
        CLOSE (0);

        public double setPos;

        ClawStates(double setPos) {
            this.setPos = setPos;
        }
    }

    Servo claw;
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        claw = hardwareMap.servo.get("clawServo");
    }
    public void setPosition(ClawStates state) {
        claw.setPosition(state.setPos);
    }

    public Action clawAction(ClawStates state) {
        return (telemetryPacket) -> {
            setPosition(state);
            return false;
        };

    }



    //TODO: Servo code that moves claw + rr action
}
