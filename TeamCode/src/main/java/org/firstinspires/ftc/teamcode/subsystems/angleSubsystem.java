package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class angleSubsystem extends SubsystemBase {

    //Motor used to change the angle of the arm
    private final Motor angleChanger;

    public angleSubsystem(final HardwareMap hmap, final String name){
        angleChanger = hmap.get(Motor.class, name);
    }
    //TODO: Figure out whether or not this is feasible
    public void angleSetTo(int degrees){
        //TODO: Some conversion of an input in DEGREES to TICKS for the motor, angleChanger, to spin for the arm to be set to a specific angle, in degrees.
        //this works so well 100% (its 2am)
        angleChanger.setTargetPosition(degrees);
    }

}
