package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

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

    MovementDirection propLocation = MovementDirection.MIDDLE;

    @Override
    public Mat processFrame(Mat input) {
        // Divide the frame into three equal sections horizontally
        int width = input.width(); // returns width of the input
        int height = input.height(); // returns height of the input
        int sectionWidth = width / 3; // Divides it into three sections

        // Define mats to be used throughout the pipeline
        Mat YcBcr = new Mat(); // Store the converted mat as YcBcr

        // Define variables for counting blue pixels in each section
        int[] bluePixelCount = new int[3]; // Makes an array that holds 3 integer values

        // Loop through each section
        for (int i = 0; i < 3; i++) {
            // Define the region of interest (ROI) for the current section
            Rect roi = new Rect(i * sectionWidth, 0, sectionWidth, height);
            Mat section = YcBcr.submat(roi);

            // Apply a color filter to detect blue regions
            Mat blueMask = new Mat();
            Core.inRange(section, new Scalar(253, 92, 76), new Scalar(203, 96, 79), blueMask);

            // Count the number of blue pixels in the current section
            bluePixelCount[i] = Core.countNonZero(blueMask);

            // Clean up resources
            blueMask.release();
            section.release();
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

        // Draw rectangles on the output to visualize the sections (optional)
        Imgproc.rectangle(input, new Point(0, 0), new Point(sectionWidth, height), new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, new Point(sectionWidth, 0), new Point(2 * sectionWidth, height), new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, new Point(2 * sectionWidth, 0), new Point(width, height), new Scalar(255, 0, 0), 2);

        // Release resources
        YcBcr.release();

        // Return the original input (you can modify this to return a processed frame)
        return input;
    }

    public MovementDirection getDirection() {
        return propLocation;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Divide the frame into two equal sections vertically
        int width = input.width(); // returns width of the input
        int height = input.height(); // returns height of the input
        int sectionHeight = height / 2; // Divides it into two sections

        // Define mats to be used throughout the pipeline
        Mat YcBcr = new Mat(); // Store the converted mat as YcBcr
        Mat output = new Mat(); // Returned at the end of the pipeline

        // Define variables for counting yellow pixels in each section
        int[] yellowPixelCount = new int[2]; // Makes an array that holds 2 integer values

        // Loop through each section
        for (int i = 0; i < 2; i++) {
            // Define the region of interest (ROI) for the current section
            Rect roi = new Rect(i * sectionHeight, 0, sectionHeight, width);
            Mat section = YcBcr.submat(roi);

            // Apply a color filter to detect yellow regions
            Mat yellowMask = new Mat();
            Core.inRange(section, new Scalar(66, 100, 100), new Scalar(44, 100, 100), yellowMask);

            // Count the number of yellow pixels in the current section
            yellowPixelCount[i] = Core.countNonZero(yellowMask);

            // Clean up resources
            yellowMask.release();
            section.release();
        }

        // Determine the section with the most yellow pixels
        boolean maxYellowIndex = false; // Assume false initially
        if (yellowPixelCount[1] > yellowPixelCount[0]) {
            maxYellowIndex = true;
        }

        // Release resources
        YcBcr.release();

        // Return the original input (you can modify this to return a processed frame)
        return input;
    }
}