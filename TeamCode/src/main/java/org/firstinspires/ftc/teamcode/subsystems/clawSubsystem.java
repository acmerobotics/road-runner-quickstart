package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawSubsystem extends SubsystemBase {

    private final Servo claw;
    //TODO: Figure out what these values are
    private final float open = 0;
    private final float closed = 1;


    //hMap is understandable, name is the name of the servo used
    public clawSubsystem(final HardwareMap hMap, final String name){
        claw = hMap.get(Servo.class, name);
    }

    public void open(){
        claw.setPosition(open);
    }

    public void close(){
        claw.setPosition(closed);
    }
}
