package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SenseHub extends Mechanism{
    private DistanceSensor temp;
    private Rev2mDistanceSensor distance;
    @Override
    public void init(HardwareMap hwMap) {
        temp = hwMap.get(DistanceSensor.class, "distance");
        distance = (Rev2mDistanceSensor)temp;
    }
    public double distance() {
        return distance.getDistance(DistanceUnit.CM);
    }

    public boolean inRange(){
        return distance() <= 43;
    }
}
