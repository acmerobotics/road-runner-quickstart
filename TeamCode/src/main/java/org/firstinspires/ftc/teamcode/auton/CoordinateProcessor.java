package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.teamcode.teleop.Robot;
import org.opencv.core.Point;

public class CoordinateProcessor {

    private openCVHomography homographyCalculator;

    public CoordinateProcessor() {
        homographyCalculator = new openCVHomography(); // Initialize the homography calculator
    }

    public void processCoordinates(double tx, double ty) {
        Point imagePoint = new Point(tx, ty);
        double[] worldCoordinates = homographyCalculator.convertTo3D(imagePoint);

        // Use the coordinates as needed
        System.out.println("Processed 3D World Coordinates: X=" + worldCoordinates[0] +
                ", Y=" + worldCoordinates[1] + ", Z=" + worldCoordinates[2]);
    }

    public static void main(String[] args) {
        CoordinateProcessor processor = new CoordinateProcessor();
        processor.processCoordinates(150, 200); // Sample tx and ty values
    }
}