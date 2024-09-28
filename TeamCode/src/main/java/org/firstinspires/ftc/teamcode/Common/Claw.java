package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {
    SimpleServo Claw;
    SimpleServo Claw2;

    public Claw(SimpleServo c, SimpleServo c2){
        Claw = c;
        Claw2 = c2;
    }

    public Claw(HardwareMap hardwareMap){
        Claw = new SimpleServo(hardwareMap, Constants.Claw1ConfigName, 0.0, 1.0);
        Claw2 = new SimpleServo(hardwareMap, Constants.Claw2ConfigName, 0.0, 1.0);
    }

    public double getClaw1Position() {
        return Claw.getPosition();
    }
    public double getClaw2Position() {
        return Claw2.getPosition();
    }
    public void ClawOpen(){
        Claw.setPosition(Constants.ClawOpen);
        Claw2.setPosition(Constants.Claw2Open);
    }
    public void Claw1Open(){
        Claw.setPosition(Constants.ClawOpen);
    }

    public void Claw2Open(){
        Claw2.setPosition(Constants.Claw2Open);
    }

    public void ClawClosed(){
        Claw.setPosition(Constants.ClawClosed);
        Claw2.setPosition(Constants.Claw2Closed);
    }
}
