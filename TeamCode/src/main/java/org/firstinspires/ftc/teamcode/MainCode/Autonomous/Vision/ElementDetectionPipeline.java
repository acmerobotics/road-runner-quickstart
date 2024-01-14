package org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

public class ElementDetectionPipeline extends OpenCvPipeline {

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    int width = VisionParameters.resX;
    int height = VisionParameters.resY;

    int minHue = 0;
    int maxHue = 255;
    int minSat = 0;
    int maxSat = 255;
    int minVal = 0;
    int maxVal = 255;

    void setColorParameters(
            int minHue,
            int maxHue,
            int minSat,
            int maxSat,
            int minVal,
            int maxVal
    ){
        this.minHue = minHue;
        this.maxHue = maxHue;
        this.minSat = minSat;
        this.maxSat = maxSat;
        this.minVal = minVal;
        this.maxVal = maxVal;
    }

    int xStart = 0;
    int yStart = 0;
    int xEnd = width;
    int yEnd = height;

    void setPositionParameters(
            int xStart,
            int yStart,
            int xEnd,
            int yEnd
    ){
        this.xStart = xStart;
        this.xEnd = xEnd;
        this.yStart = yStart;
        this.yEnd = yEnd;
    }



    double amount;


    @Override
    public Mat processFrame(Mat image) {


        Mat submat = image.submat(yStart, yEnd, xStart, xEnd);
        Mat thresh = submat.clone();
        Imgproc.cvtColor(submat, thresh, Imgproc.COLOR_BGR2HSV);
        Core.inRange(thresh, new Scalar(minHue, minSat, minVal), new Scalar(maxHue, maxSat, maxVal), thresh);

        amount = Core.sumElems(thresh).val[0]/255./(xEnd - xStart)/(yEnd - yStart);


        Mat thresh4 = new Mat();
        Imgproc.cvtColor(thresh, thresh4, Imgproc.COLOR_GRAY2BGRA);
        Core.add(submat, thresh4, submat);

        Imgproc.rectangle(
                image,
                new Point(xStart, yStart),
                new Point(xEnd, yEnd),
                new Scalar(255, 255, 255),
                5
        );

        thresh.release();
        thresh4.release();

        return image;
    }
}
