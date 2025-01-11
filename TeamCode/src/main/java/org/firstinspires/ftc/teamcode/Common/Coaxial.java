package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Coaxial {
    SimpleServo CoAxial;

    public Coaxial(HardwareMap hardwareMap){

        CoAxial = new SimpleServo(hardwareMap, "Coax", 0.0, 1.0);
    }
    public void CoAxback() { CoAxial.setPosition(0.8);};

    public void Coaxfront() {CoAxial.setPosition(0.2);}
        };


