package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Config
public class SenseHub extends Mechanism{
    private DistanceSensor temp;
    private Rev2mDistanceSensor distance;
    public static double range = 43;
    @Override
    public void init(HardwareMap hwMap) {
        temp = hwMap.get(DistanceSensor.class, "sens");
        distance = (Rev2mDistanceSensor)temp;

    }
    public double distance() {
        return distance.getDistance(DistanceUnit.CM);
    }

    public boolean inRange(){
        return distance() <= range;
    }
}
