package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


/**
 * Used to detect cone position, and sleeve color.
 */
public class ObjectDetection extends OpenCvPipeline {

    boolean debug = false;

    public boolean stopConeDetectPipeLine = true;

    // for sleeve color detect
    public enum ParkingLot {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    // TOPLEFT anchor point for the bounding box
    private final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(20, 140);

    // Width and height for the bounding box
    private final int SLEEVE_REGION_WIDTH = 30;
    private final int SLEEVE_REGION_HEIGHT = 50;

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + SLEEVE_REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + SLEEVE_REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingLot parkingLot = ParkingLot.UNKNOWN;
    private volatile double parkingLotDistance = 0.0;
    private volatile boolean coneDetected = false;
    // Running variable storing the parking position
    private volatile double objectPosition = 0;
    public volatile double hChannelAve = 0;

    //check if cone has been detected.
    private final double cameraToConeDistance = 13.0; // inch
    private final double cameraViewAngle = 0.67;

    // TOP-LEFT anchor point for the bounding box
    public Point Object_TOPLEFT_POINT = new Point(0, 0);

    // Width and height for the bounding box
    public int REGION_WIDTH = 320;
    public int REGION_HEIGHT = 240;

    // Color definitions
    private final Scalar
            RED   = new Scalar(255, 0, 0),
            GREEN = new Scalar(0, 255, 0),
            BLUE  = new Scalar(0, 0, 255);
    private Scalar brushColor = new Scalar(0, 0, 0);

    // Anchor point definitions
    Point pointA = new Point(
            Object_TOPLEFT_POINT.x,
            Object_TOPLEFT_POINT.y);
    Point pointB = new Point(
            Object_TOPLEFT_POINT.x + REGION_WIDTH,
            Object_TOPLEFT_POINT.y + REGION_HEIGHT);


    @Override
    public Mat processFrame(Mat input) {
        // detect sleeve color.
        sleeveColorDetect(input);

        // detect cone location
        if (!stopConeDetectPipeLine) {
            Logging.log("Start Opcv process to detect cone.");
            conePositionDetect(input);
        }

        // draw a rectangle on sleeve.
        Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    brushColor,
                    2
        );

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public double getConePosition() {
        return objectPosition;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingLot getParkingLot() {
        return parkingLot;
    }

    // Returns an enum being the current position where the robot will park
    public double getParkingLotDistance() {
        return parkingLotDistance;
    }

    // Returns an enum being the current position where the robot will park
    public boolean isConeDetected() {
        return coneDetected;
    }

    private void sleeveColorDetect(Mat sleeveImageInput) {
        Logging.log("Start Opcv process to detect sleeve color.");
        // Get the submat frame, and then sum all the values
        Mat areaMat = sleeveImageInput.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        // Get the minimum RGB value from every single channel
        double maxColor = Math.max(sumColors.val[0], Math.max(sumColors.val[1], sumColors.val[2]));
        Logging.log("Sleeve max color = %.2f, %.2f, %.2f", sumColors.val[0], sumColors.val[1], sumColors.val[2]);

        // Change the bounding box color based on the sleeve color
        if (maxColor < Math.ulp(0)){
            parkingLot = ParkingLot.UNKNOWN;
        }
        else if (Math.abs(sumColors.val[0] - maxColor) < Math.ulp(0)) {
            parkingLot = ParkingLot.LEFT;
            brushColor = RED;
            parkingLotDistance = -24;
        } else if (Math.abs(sumColors.val[1] - maxColor) < Math.ulp(0)) {
            parkingLot = ParkingLot.CENTER;
            brushColor = GREEN;
            parkingLotDistance = 0;
        } else {
            parkingLot = ParkingLot.RIGHT;
            brushColor = BLUE;
            parkingLotDistance = 24;
        }
        Logging.log("Sleeve position: %s", parkingLot.toString());

        // Release and return input
        areaMat.release();
    }

    private void conePositionDetect(Mat inputCone) {
        // Get the submat frame, and then sum all the values

        Mat areaMat = inputCone.submat(new Rect(pointA, pointB));

        Mat doubleAreaMat = new Mat(areaMat.size(), 30); // CV_64FC3, CV_64FC4=30
        areaMat.convertTo(doubleAreaMat, 30);

        int kernelSize = 32;
        Mat destMat = new Mat(doubleAreaMat.rows(), doubleAreaMat.cols(), doubleAreaMat.type());

        double [] filterK = new double[doubleAreaMat.channels()];
        for (int i = 0; i < doubleAreaMat.channels(); i++)
        {
            filterK[i] = 1.0/kernelSize/kernelSize;
        }

        Mat kernel = new Mat(kernelSize, kernelSize, doubleAreaMat.type() ) {
            {
                for (int i = 0; i < kernelSize; i++) {
                    for (int j = 0; j < kernelSize; j++) {
                        put(i, j, filterK);
                    }
                }
            }
        };

        Imgproc.filter2D(doubleAreaMat, destMat, -1, kernel);

        Mat lineMatA = destMat.row(0);
        Mat lineM = new Mat(lineMatA.size(), lineMatA.type());
        for (int i = 1; i < destMat.rows(); i++) {
            Mat lineMatB = destMat.row(i);
            Core.add(lineMatA, lineMatB, lineM);
            lineMatA = lineM;
        }

        // lineM should only have one row.
        Mat lineRed = new Mat();
        Mat lineBlue = new Mat();
        Mat lineSum = new Mat();
        Core.extractChannel(lineM, lineRed, 0);
        Core.extractChannel(lineM, lineBlue, 0);
        Core.add(lineRed, lineBlue, lineSum); // add or subtract??
        double maxPixelVal = 0.0, minPixelVal = lineM.get(0, 0)[0];
        int maxPixelLoc = 0;

        // find max value and its index in array for each channel
        for (int j = 0; j < lineSum.cols(); j++) {
            double pixelChannelVal = Math.abs(lineSum.get(0, j)[0]);
            if (maxPixelVal < pixelChannelVal) {
                maxPixelVal = pixelChannelVal;
                maxPixelLoc = j;
            }

            if (minPixelVal > pixelChannelVal) {
                minPixelVal = pixelChannelVal;
            }

            if (debug)
                Logging.log("Pixel values(%d, %d) = %.2f", 0, j, lineM.get(0, j)[0]);

        }

        coneDetected = verifyObjectDetected(lineM, maxPixelLoc);

        if (coneDetected) {
            objectPosition = (maxPixelLoc * 2.0 / lineM.cols()) * Math.tan(cameraViewAngle / 2) * cameraToConeDistance;
            Imgproc.line(inputCone, new Point(maxPixelLoc, 0), new Point(maxPixelLoc, inputCone.rows()), GREEN);
        }
        else {
            objectPosition = 0.0;
        }
        if (debug)
            Logging.log(" location pixel is %d, distance is %.2f", maxPixelLoc, objectPosition);

        Imgproc.rectangle(
                inputCone,
                pointA,
                pointB,
                GREEN,
                2
        );

        // Release and return input
        areaMat.release();
        lineM.release();
        lineMatA.release();
        kernel.release();
        doubleAreaMat.release();
        destMat.release();
        lineRed.release();
        lineBlue.release();
        lineSum.release();
    }

    private void checkColorByHSV(Mat m) {

        Mat hsvFrame, rgbFrame;//, inRangeMask, filteredFrame, hChannel;
        rgbFrame = new Mat();
        hsvFrame = new Mat();
        // Convert the frame in the HSV color space, to be able to identify the color with the thresholds
        Imgproc.cvtColor(m, rgbFrame, Imgproc.COLOR_RGBA2RGB); // Cant't convert directly rgba->hsv
        Imgproc.cvtColor(rgbFrame, hsvFrame, Imgproc.COLOR_RGB2HSV);
        Scalar hsvColors = Core.sumElems(hsvFrame);
        hChannelAve = hsvColors.val[0] / m.rows() / m.cols();

        if (hChannelAve > Math.ulp(0)) {
            if (hChannelAve < 30 || hChannelAve > 130) {
                parkingLot = ParkingLot.LEFT;
                brushColor = RED;
                parkingLotDistance = -24.0;
            } else if (hChannelAve >= 30 && hChannelAve <= 100) {
                parkingLot = ParkingLot.CENTER;
                brushColor = GREEN;
                parkingLotDistance = 0.0;
            } else {
                parkingLot = ParkingLot.RIGHT;
                brushColor = BLUE;
                parkingLotDistance = 24.0;
            }
        }
        Logging.log(" hChannelAve = %.2f", hChannelAve);
        rgbFrame.release();
        hsvFrame.release();
    }

    private boolean verifyObjectDetected(@NonNull Mat lineM, int maxPixelLoc)
    {
        int colsNum = lineM.cols();
        int  minPixelLoc;

        int tmp = maxPixelLoc + colsNum/2;
        minPixelLoc = (tmp > colsNum)? (tmp - colsNum) : tmp;


        Mat maxPixels;
        Mat minPixels;

        if ((maxPixelLoc-10) <= 0) {
            maxPixels = lineM.colRange(maxPixelLoc, maxPixelLoc+10);
        }
        else
        {
            maxPixels = lineM.colRange(maxPixelLoc-10, maxPixelLoc);
        }

        if ((minPixelLoc-10) <= 0) {
            minPixels = lineM.colRange(minPixelLoc, minPixelLoc+10);
        }
        else
        {
            minPixels = lineM.colRange(minPixelLoc-10, minPixelLoc);
        }

        Scalar sumMaxColor = Core.sumElems(maxPixels);
        Scalar sumMinColor = Core.sumElems(minPixels);

        double[] per = new double[3];
        double minRatioChannel = 1.0;
        for(int j = 0; j < 3; j++) {
            per[j] = sumMinColor.val[j] / sumMaxColor.val[j];
            if (debug) {
                Logging.log("per [%d] = %.3f, cone location = %d", j, per[j], maxPixelLoc);
            }
            if (per[j] < minRatioChannel) {
                minRatioChannel = per[j];
            }
        }
        return minRatioChannel < 0.66;
    }
}
