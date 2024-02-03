package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class cameraPipeline extends cameraInitialization {

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
        Mat output = new Mat(); // Returned at the end of the pipeline

        // Define variables for counting red pixels in each section
        int[] redPixelCount = new int[3]; // Makes an array that holds 3 integer values

        // Loop through each section
        for (int i = 0; i < 3; i++) {
            // Define the region of interest (ROI) for the current section
            Rect roi = new Rect(i * sectionWidth, 0, sectionWidth, height);
            Mat section = YcBcr.submat(roi);

            // Apply a color filter to detect red regions
            Mat redMask = new Mat();
            Core.inRange(section, new Scalar(0, 100, 100), new Scalar(10, 255, 255), redMask);
            Core.inRange(section, new Scalar(160, 100, 100), new Scalar(179, 255, 255), redMask);

            // Count the number of red pixels in the current section
            redPixelCount[i] = Core.countNonZero(redMask);

            // Clean up resources
            redMask.release();
            section.release();
        }

        // Determine the section with the most red pixels
        int maxRedIndex = 1; // Assume the middle section initially
        if (redPixelCount[0] > redPixelCount[1] && redPixelCount[0] > redPixelCount[2]) {
            maxRedIndex = 0;
        } else if (redPixelCount[2] > redPixelCount[0] && redPixelCount[2] > redPixelCount[1]) {
            maxRedIndex = 2;
        }

        // Determine the movement direction based on the section with the most red pixels
        switch (maxRedIndex) {
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
}