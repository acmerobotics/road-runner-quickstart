package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class FreightSensor extends Mechanism{
    ColorSensor color;
    @Override
    public void init(HardwareMap hwMap) {
        color = hwMap.get(ColorSensor.class, "color");
    }
    public boolean hasFreight() {
        return ((color.red() + color.green()) / 2 > color.blue() && (color.red()+color.green()) / 2 >= 100); //idk how this works mannn
    }
}
