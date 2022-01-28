package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class FreightSensor extends Mechanism{
    ColorSensor left;
    ColorSensor right;
    @Override
    public void init(HardwareMap hwMap) {

        left = hwMap.get(ColorSensor.class, "color1");
    }
    public boolean hasFreightSensor(ColorSensor color) {
        return ((color.red() + color.green()) / 2 > color.blue() && (color.red()+color.green()) / 2 >= 100); //idk how this works mannn
    }

    public boolean hasFreight(){
        return hasFreightSensor(left);
    }

}
