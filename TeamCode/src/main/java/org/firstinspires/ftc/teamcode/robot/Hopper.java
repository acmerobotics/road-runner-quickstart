package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Map;

public class Hopper {
    private Map stateMap;
    private Telemetry telemetry;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    public final String HOPPER_SYSTEM_NAME = "HOPPER_SYSTEM_NAME";
    public final String HOPPER_NO_PIXELS = "HOPPER_NO_PIXEL";
    public final String HOPPER_ONE_PIXEL = "HOPPER_ONE_PIXEL";
    public final String HOPPER_TWO_PIXELS = "HOPPER_TWO_PIXELS";
    private final float gain = 10;

    private Constants constants = new Constants();
    ElapsedTime waitTime = new ElapsedTime();

    public Hopper(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        colorSensor1 = hwMap.get(NormalizedColorSensor.class, "bottomHopperColorSensor");
        colorSensor2 = hwMap.get(NormalizedColorSensor.class, "upperHopperColorSensor");

        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);

    }

    public void setState(){
        telemetry.addData("Color sensor 1 distance", (((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM)));
        telemetry.addData("Color sensor 2 distance", (((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM)));
        boolean pixelSensor1 = (((DistanceSensor) colorSensor1).getDistance(DistanceUnit.CM)) < 0.65;
        boolean pixelSensor2 = (((DistanceSensor) colorSensor2).getDistance(DistanceUnit.CM)) < 0.65;

        if(!pixelSensor1){
            stateMap.put(HOPPER_SYSTEM_NAME, HOPPER_NO_PIXELS);
        }

        if(pixelSensor1 && !pixelSensor2) {
            stateMap.put(HOPPER_SYSTEM_NAME, HOPPER_ONE_PIXEL);
            if (stateMap.get(constants.HOPPER_2_PIXELS_TIME) == null) {
                stateMap.put(constants.HOPPER_2_PIXELS_TIME, System.currentTimeMillis());
            }
        }

        if(stateMap.get(constants.HOPPER_2_PIXELS_TIME) !=  null){
            long hopperEndTime = (long) stateMap.get(constants.HOPPER_2_PIXELS_TIME) + 250;
            telemetry.addData("Current time", System.currentTimeMillis());
            telemetry.addData("End time", hopperEndTime);
            if(System.currentTimeMillis() > hopperEndTime && pixelSensor2){
                stateMap.put(HOPPER_SYSTEM_NAME, HOPPER_TWO_PIXELS);
                stateMap.put(constants.HOPPER_2_PIXELS_TIME, null);
            }
        }
    }
}
