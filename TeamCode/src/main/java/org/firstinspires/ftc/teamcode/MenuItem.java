package org.firstinspires.ftc.teamcode;

public class MenuItem {
    double value;
    String label;
    double max;
    double min;
    double increment;

    public MenuItem(double v, String L, double M, double m, double i) {
        value = v;
        label = L;
        max = M;
        min = m;
        increment = i;
    }

    public MenuItem() {
        value = 0.0;
        label = "menu item";
        max = 1.0;
        min = 0.0;
        increment = 0.1;
    }

    public void up(){
        if (value + increment < max) {
            value += increment;
        }
        else value = max;
    }

    public void down() {
        if (value - increment > min) {
            value -= increment;
        }
        else value = min;
    }

    public double getValue() {
        return value;
    }

    public String getLabel() {
        return label;
    }

    public String getRepr() {
        return label + ": " + value;
    }
}
