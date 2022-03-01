package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.lang.Math;

@Config
public class FreightSensor extends Mechanism{
    // Adaptar class for color sensor that detects freight

    //ranges
    public static double lowB = 1.5;
    public static double HighB = 3.5;

    //can utilize more than one color sensor
    ColorSensor left;
    ColorSensor right;
    @Override
    public void init(HardwareMap hwMap) {

        left = hwMap.get(ColorSensor.class, "color1");
    }

    /**
     *
     * @param color ColorSensor to which the logic is applied
     * @return a boolean that returns whether or not a freight is detected
     */
    public boolean hasFreightSensor(ColorSensor color) {
        return ((color.red() + color.green()) / 2 > color.blue() && (color.red()+color.green()) / 2 >= 100); //idk how this works mannn
    }

    /**
     *
     * @return returns the status of detected freight
     */
    public boolean hasFreight(){
        return hasFreightSensor(left);
    }

    /**
     *
     * @return green component of color sensor
     */
    public double green() {
        return left.green();
    }

    /**
     *
     * @return red componenet of color sensor
     */
    public double red() {
        return left.red();
    }

    /**
     * blue componenet of color sensor
     * @return
     */
    public double blue() {
        return left.blue();
    }

    public boolean hasBlock() {
        return (this.red() / this.blue()) >= lowB && (this.red() / this.blue() <= HighB);
    }

    public boolean hasBall() {
    	return Math.round((this.red()/this.blue())) == 1 && Math.round(this.red()/this.green()) == 1 && Math.round(this.blue()/this.green()) == 1;
    }
}
