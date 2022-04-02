package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    public static double GAINR = 0.2;
    LowPassFilter lowPassFilterR = new LowPassFilter(GAINR);

    public static double GAINB = 0.2;
    LowPassFilter lowPassFilterB = new LowPassFilter(GAINB);

    public static double GAING = 0.2;
    LowPassFilter lowPassFilterG = new LowPassFilter(GAING);

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
        //return ((color.red() + color.green()) / 2 > color.blue() && (color.red()+color.green()) / 2 >= 100); //idk how this works mannn
        return  (green() + blue() + red()) / 3 > 100;
    }

    public boolean hasFreightKalmanSensor(ColorSensor color) {
        //return ((color.red() + color.green()) / 2 > color.blue() && (color.red()+color.green()) / 2 >= 100); //idk how this works mannn
        return  (greenEstimate() + blueEstimate() + redEstimate()) / 3 > 100;
    }

    /**
     *
     * @return returns the status of detected freight
     */
    public boolean hasFreight(){
        return hasFreightKalmanSensor(left);
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


    public double redEstimate(){
        return getLowPass(red(),lowPassFilterR);
    }


    public double blueEstimate(){
        return getLowPass(blue(),lowPassFilterB);
    }

    public double greenEstimate(){
        return getLowPass(green(),lowPassFilterG);
    }

    public boolean hasBlockKalman(){
        return (this.redEstimate() / this.blueEstimate()) >= lowB && (this.redEstimate() / this.blueEstimate() <= HighB);

    }
    public double getLowPass(double value, LowPassFilter filter) {
        double currentValue = value;
        double estimate = filter.estimate(currentValue);
        return estimate;
    }

    public boolean hasBlock() {
        return (this.red() / this.blue()) >= lowB && (this.red() / this.blue() <= HighB);
    }

    public boolean hasBall() {
    	return Math.round((this.red()/this.blue())) == 1 && Math.round(this.red()/this.green()) == 1 && Math.round(this.blue()/this.green()) == 1;
    }
}
