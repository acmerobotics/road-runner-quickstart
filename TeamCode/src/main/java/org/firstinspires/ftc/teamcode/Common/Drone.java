package org.firstinspires.ftc.teamcode.Common;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drone {
    SimpleServo drone;

    public Drone(SimpleServo d){
        drone = d;
    }

    public Drone(HardwareMap hardwareMap){
        drone = new SimpleServo(hardwareMap, Constants.DroneConfigName, 0.0, 1.0);
    }

    public void resetDrone() {
        drone.setPosition(Constants.DroneReset);
    }

    public void flyDrone() {
        drone.setPosition(Constants.DroneFly);
    }
}
