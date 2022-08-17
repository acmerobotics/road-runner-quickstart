package org.firstinspires.ftc.teamcode.Math.Geometry;

import static java.lang.Math.toDegrees;

public class Angle {
    
    protected final double ARC_SECONDS_CONSTANT = 3600;
    protected double radians;
    protected double degrees;
    protected double arcSeconds;

    /**
     * radians constructor 
     * @param radians angle in radians
     */
    public Angle(double radians) {
        this.radians = radians;
        this.degrees = toDegrees(radians);
        this.arcSeconds = degrees * ARC_SECONDS_CONSTANT;
    }


    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return degrees;
    }

    public void setRadians(double radian) {
        this.radians = radian;
        this.degrees = toDegrees(radian);
        this.arcSeconds = toDegrees(radian) * ARC_SECONDS_CONSTANT;
    }


}
