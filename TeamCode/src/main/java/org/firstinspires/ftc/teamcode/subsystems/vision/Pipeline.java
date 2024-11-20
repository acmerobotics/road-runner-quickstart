package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    public Scalar redLowHSV = new Scalar(0, 50, 80);
    public Scalar redHighHSV = new Scalar(25, 230, 255);
    public Scalar blueLowHSV = new Scalar(80, 70, 55);
    public Scalar blueHighHSV = new Scalar(255, 255, 255);

    public double tolerance = 0.05;

    boolean isRedAlliance;
    Mat mat = new Mat();
    Scalar lowHSV;
    Scalar highHSV;
    public static int rightRectX = 500;
    public static int rightRectY = 80;
    public static int leftRectX = 50;
    public static int leftRectY = 80;
    public static int centerRectX = 270;
    public static int centerRectY = 50;
    Rect RIGHT_RECT, CENTER_RECT, LEFT_RECT;
    double rightRegionPercent, centerRegionPercent, leftRegionPercent;
    int region = 2;

    public Pipeline() {
        this.isRedAlliance = true;
    }

    public Pipeline(Boolean isRedAlliance) {
        this.isRedAlliance = isRedAlliance;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (isRedAlliance) {
            lowHSV = redLowHSV;
            highHSV = redHighHSV;
        } else {
            lowHSV = blueLowHSV;
            highHSV = blueHighHSV;
        }

        RIGHT_RECT = new Rect(rightRectX, rightRectY, 120, 120);
        CENTER_RECT = new Rect(centerRectX, centerRectY, 120, 120);
        LEFT_RECT = new Rect(leftRectX, leftRectY, 120, 120);

        Core.inRange(mat, lowHSV, highHSV, mat);

        // submats for the boxes, these are the regions that'll detect the color
        Mat rightBox = mat.submat(RIGHT_RECT);
        Mat centerBox = mat.submat(CENTER_RECT);
        Mat leftBox = mat.submat(LEFT_RECT);

        // how much in each region is white aka the color we filtered through the mask
        rightRegionPercent = Core.sumElems(rightBox).val[0] / RIGHT_RECT.area() / 255;
        centerRegionPercent = Core.sumElems(centerBox).val[0] / CENTER_RECT.area() / 255;
        leftRegionPercent = Core.sumElems(leftBox).val[0] / LEFT_RECT.area() / 255;

        Imgproc.rectangle(mat, LEFT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(60, 255, 255), 5);
        Imgproc.rectangle(mat, CENTER_RECT, new Scalar(60, 255, 255), 5);

        Imgproc.putText(mat, "Left: " + String.format("%.4f", leftRegionPercent), new org.opencv.core.Point(0, 25), 0, 1, new Scalar(180, 255, 255), 2);
        Imgproc.putText(mat, "Center: " + String.format("%.4f", centerRegionPercent), new org.opencv.core.Point(0, 50), 0, 1, new Scalar(180, 255, 255), 2);
        Imgproc.putText(mat, "Right: " + String.format("%.4f", rightRegionPercent), new org.opencv.core.Point(0, 75), 0, 1, new Scalar(180, 255, 255), 2);

        if (leftRegionPercent > tolerance && leftRegionPercent > centerRegionPercent && leftRegionPercent > rightRegionPercent) {
            Imgproc.rectangle(mat, LEFT_RECT, new Scalar(180, 255, 255), 10);
            region = 1;
        } else if (rightRegionPercent > tolerance && rightRegionPercent > centerRegionPercent && rightRegionPercent > leftRegionPercent) {
            Imgproc.rectangle(mat, RIGHT_RECT, new Scalar(180, 255, 255), 10);
            region = 3;
        } else { // (centerRegionPercent > tolerance && centerRegionPercent > rightRegionPercent && centerRegionPercent > leftRegionPercent) {
            Imgproc.rectangle(mat, CENTER_RECT, new Scalar(180, 255, 255), 10);
            region = 2;
        }

        rightBox.release();
        leftBox.release();
        centerBox.release();

        return mat;
    }

    public int whichRegion() {
        return region;
    }
}