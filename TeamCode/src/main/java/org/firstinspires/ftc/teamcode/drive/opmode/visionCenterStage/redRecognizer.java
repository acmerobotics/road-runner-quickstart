package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class redRecognizer extends OpenCvPipeline {
    public enum pixelLocationRed {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN,
    }
    // Creates our enum's that the program returns at the end
    private final Mat YcBcR = new Mat();
    //This is the mat that receives the camera output after a Grayscale filter is applied
    private Mat leftMat = new Mat();
    private Mat middleMat = new Mat();
    private Mat rightMat = new Mat();
    //Individual matrices to detect different sections of the screen, each getting 1/3rd respectively,
    //with the left getting slightly more due to camera positioning
    private double leftAvg, middleAvg, rightAvg;
    //Creates our variables to collect the average number of pixels turned white after Grayscale filter is applied
    private final Mat output = new Mat();
    //Mat thats there so I can see unGrayscaled filtered image whenever necessary.
    private final Rect leftRect = new Rect(0,100,140,80);
    private final Rect middleRect = new Rect(560,0,200,100);
    private final Rect rightRect = new Rect(1100,100,140,80);
    //Rectangles created to define where leftMat, rightMat, & middleMat check for correctly colored pixels.
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YcBcR, Imgproc.COLOR_RGB2GRAY);
       //converts image to grayscale
        input.copyTo(output);
        YcBcR.copyTo(leftMat);
        YcBcR.copyTo(middleMat);
        YcBcR.copyTo(rightMat);
        // copies grayscale to all inputs and output screen

        leftMat = input.submat(leftRect); //defines left mat's location
        Scalar avgIntensityLeft = Core.mean(leftMat); //Detects the average intensity of left mat and stores it as a scalar
        leftAvg = avgIntensityLeft.val[0]; //stores the Scalar value as a variable

        middleMat = input.submat(middleRect); //defines middle mat's location
        Scalar avgIntensityMiddle = Core.mean(middleMat); //Detects the average intensity of middle mat and stores it as a scalar
        middleAvg = avgIntensityMiddle.val[0]; //stores the Scalar value as a variable

        rightMat = input.submat(rightRect); //defines right mat's location
        Scalar avgIntensityRight = Core.mean(rightMat); //Detects the average intensity of right mat and stores it as a scalar
        rightAvg = avgIntensityRight.val[0]; //stores the Scalar value as a variable

        Imgproc.rectangle(output, leftRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(output, middleRect, new Scalar(255.0, 255.0, 0.0), 2);
        Imgproc.rectangle(output, rightRect, new Scalar(255.0, 255.0, 0.0), 2);
        // Creates bounding boxes on the output for diagnostic reasons

        Imgproc.putText(output, getPixelLocationRed().name(), new Point(25,100), Imgproc.FONT_HERSHEY_SIMPLEX,3.0,new Scalar(0.0, 255.0, 0.0));
        //places the PixelLocation on the camera stream.
        return output;

    }
    public pixelLocationRed getPixelLocationRed() {
        if (leftAvg > middleAvg && leftAvg > rightAvg) {
            return pixelLocationRed.LEFT;
        } else if (middleAvg > leftAvg && middleAvg > rightAvg) {
            return pixelLocationRed.MIDDLE;
        } else if (rightAvg > leftAvg && rightAvg > middleAvg) {
                return pixelLocationRed.RIGHT;
        } else {
            return pixelLocationRed.UNKNOWN;
        }
        //Basic if statement that compares the average intensities of each mat and returns which had the highest
    }
}
