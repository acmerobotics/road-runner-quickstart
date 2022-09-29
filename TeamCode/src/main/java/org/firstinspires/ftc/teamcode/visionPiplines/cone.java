package org.firstinspires.ftc.teamcode.visionPiplines;

import org.opencv.core.MatOfPoint;

public class cone {
    Color color;
    double xPos;
    double yPos;
    double area;
    double perimeter;
    double ratio;
    MatOfPoint contour;
    public cone(Color color, double xPos, double yPos, double area, double perimeter, double ratio, MatOfPoint contour) {
        this.color = color;
        this.xPos = xPos;
        this.yPos = yPos;
        this.area = area;
        this.perimeter = perimeter;
        this.ratio = ratio;
        this.contour = contour;
    }
}