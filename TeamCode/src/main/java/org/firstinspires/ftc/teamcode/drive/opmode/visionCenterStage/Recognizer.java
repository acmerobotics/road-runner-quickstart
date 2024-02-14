package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Recognizer extends OpenCvPipeline {
    public enum pixelLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN,
    }
    // Creates our enum's that the program returns at the end
    private final Mat YcBcR = new Mat();
    //This is the mat that recieves the camera output after a YcBcR filter is applied
    private Mat leftMat = new Mat();
    private Mat middleMat = new Mat();
    private Mat rightMat = new Mat();
    //Individual matrices to detect different sections of the screen, each getting 1/3rd respectively,
    //with the left getting slightly more due to camera positioning
    private double leftAvg, middleAvg, rightAvg;
    //Creates our variables to collect the average number of pixels turned white after YcBcR filter is applied
    private final Mat output = new Mat();
    //Matrix to collect our outputs and make my life easy
    private final Rect leftRect = new Rect(90,360,330,360);
    private final Rect middleRect = new Rect(420,250,440,250);
    private final Rect rightRect = new Rect(950,360,330,360);
    //Rectangles created to define scope of leftMat, middleMat, & rightMat used below
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YcBcR, Imgproc.COLOR_RGB2YCrCb);
        //Actually applying YcBcR filter using Imgproc filters that come as part of EasyOpenCv, with the output reading as YcBcR

        leftMat = input.submat(leftRect); // Defines where leftMat actually is using leftRect
        Core.inRange(YcBcR, new Scalar(0.0, 0.0, 148.0), new Scalar(230, 200.0, 255.0), leftMat);
        //Takes YcBcR's output, and analyzes the pixels within the previously defined leftRect for YcBcR values that fall
        //within the defined scalar values, (Roughly the entire blue spectrum) and returns a 1 for each pixel within that value range

        leftAvg = Core.mean(leftMat).val[0]*100.0;
        // takes that value outputted by the previous function, averages it, and multiplies by 100,000
        // this avoids the many basically zero numbers we would have to read later on, simplifying the computers comparison

        middleMat = input.submat(middleRect);
        Core.inRange(YcBcR, new Scalar(0.0, 0.0, 148.0), new Scalar(230, 200.0, 255.0), middleMat);
        //All of this is just the same as for the left Section.

        middleAvg = Core.mean(middleMat).val[0]*100.0;

        rightMat = input.submat(rightRect);
        Core.inRange(YcBcR, new Scalar(0.0, 0.0, 148.0), new Scalar(230, 200.0, 255.0), rightMat);
        // Same as for the left section again

        rightAvg = Core.mean(rightMat).val[0]*100.0;

        input.copyTo(output);

        Imgproc.rectangle(output, leftRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, middleRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, rightRect, new Scalar(255.0, 0.0, 0.0), 2);
        //Places rectangle bounding boxes on defined rectangles for diagnostic purposes

        Imgproc.putText(output, getPixelLocation().name(), new Point(25,100), Imgproc.FONT_HERSHEY_SIMPLEX,3.0,new Scalar(0.0, 255.0, 0.0));
        //Creates output text on driver hub for testing purposes
        return YcBcR;
        //puts back out image for diagnostic purposes
    }
    public pixelLocation getPixelLocation() {
        if (leftAvg > middleAvg && leftAvg > rightAvg) {
            return pixelLocation.LEFT;
        } else if (middleAvg > leftAvg && middleAvg > rightAvg) {
            return pixelLocation.MIDDLE;
        } else if (rightAvg > leftAvg && rightAvg > middleAvg) {
                return pixelLocation.RIGHT;
        } else {
            return pixelLocation.UNKNOWN;
        }
        //Basic if statement that just compares the values of each average, and the highest one gets returned.
    }
}
