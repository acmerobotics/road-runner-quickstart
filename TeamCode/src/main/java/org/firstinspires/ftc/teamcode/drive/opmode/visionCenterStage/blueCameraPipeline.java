package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;


import static java.lang.Thread.sleep;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


// https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/pipelines_overview.md
public class blueCameraPipeline extends OpenCvPipeline {
    public enum MovementDirection {
        LEFT,
        MIDDLE,
        RIGHT
    }
    
    // Use YcBcR 
    // Output different mats / masks

    private Mat sect = new Mat();
    private Mat section = new Mat();
    private Mat blueMask = new Mat();

    MovementDirection propLocation = MovementDirection.MIDDLE;
    @Override
    public Mat processFrame(Mat input) {

        // Divide the frame into three equal sections horizontally
        int width = input.width(); // returns width of the input
        int height = input.height(); // returns height of the input
        int sectionWidth = (width / 3); // Divides it into three sections

        // Define mats to be used throughout the pipeline

        input.copyTo(sect);

        // Define variables for counting blue pixels in each section
        int[] bluePixelCount = new int[3]; // Makes an array that holds 3 integer values

        // Loop through each section
        for (int i = 0; i < 3; i++) {
            // Define the region of interest (ROI) for the current section
            Rect roi = new Rect(i * sectionWidth, 0, sectionWidth, height);

            section = sect.submat(roi);

            // Apply a color filter to detect blue regions
            blueMask = new Mat();
            Core.inRange(section, new Scalar(0, 0, 100), new Scalar(80, 80, 255), blueMask);                // Count the number of blue pixels in the current section
            bluePixelCount[i] = Core.countNonZero(blueMask);

            // Clean up resources
//                blueMask.release();
//                section.release();
        }

        // Determine the section with the most blue pixels
        int maxBlueIndex = 1; // Assume the middle section initially
        if (bluePixelCount[0] > bluePixelCount[1] && bluePixelCount[0] > bluePixelCount[2]) {
            maxBlueIndex = 0;
        } else if (bluePixelCount[2] > bluePixelCount[0] && bluePixelCount[2] > bluePixelCount[1]) {
            maxBlueIndex = 2;
        }

        // Determine the movement direction based on the section with the most blue pixels
        switch (maxBlueIndex) {
            case 0:
                propLocation = MovementDirection.LEFT;
                break;
            case 1:
                propLocation = MovementDirection.MIDDLE;
                break;
            case 2:
                propLocation = MovementDirection.RIGHT;
                break;
        }
            // Return the original input (you can modify this to return a processed frame)

            Imgproc.putText(sect, "Direction: " + String.valueOf(maxBlueIndex), new Point(25, 100),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 255.0, 0.0));
            Imgproc.putText(sect, "Bluestuff: " + String.valueOf(bluePixelCount), new Point(300, 300),
                                        Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 255.0, 0.0));

            return sect;
    }

    public MovementDirection getDirection() {
        return propLocation;
    }
}
