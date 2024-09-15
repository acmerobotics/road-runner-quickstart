package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class extendSubsystem extends SubsystemBase {

    //Motor used to change the angle of the arm
    private final Motor extender;

    public extendSubsystem(final HardwareMap hmap, final String name){
        extender = hmap.get(Motor.class, name);
    }
    //TODO: Figure out whether or not this is feasible
    public void extendIn(int inches){
        //TODO: Some conversion of an input in INCHES to TICKS so that the motor, extender, lifts the arm system up INCHES.
        //this works so well 100% (its 2am)
        extender.setTargetPosition(inches);
    }
}
