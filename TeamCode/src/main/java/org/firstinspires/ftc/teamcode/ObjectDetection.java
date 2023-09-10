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

    // Enum that represents what side the prop is at
    public enum PropSide {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    // TOPLEFT anchor point for the 3 bounding boxes
    private final Point BOX_ANCHOR_ONE = new Point(20, 100);
    private final Point BOX_ANCHOR_TWO = new Point(140, 30);
    private final Point BOX_ANCHOR_THREE = new Point(260, 100);

    // Width and height for the bounding boxes
    private final int BOX_WIDTH = 40;
    private final int BOX_HEIGHT = 60;

    // Anchor point definitions for 1st box
    Point prop_pointA_one = new Point(
            BOX_ANCHOR_ONE.x,
            BOX_ANCHOR_ONE.y);
    Point prop_pointB_one = new Point(
            BOX_ANCHOR_ONE.x + BOX_WIDTH,
            BOX_ANCHOR_ONE.y + BOX_HEIGHT);

    // Anchor point definitions for 2nd box
    Point prop_pointA_two = new Point(
            BOX_ANCHOR_TWO.x,
            BOX_ANCHOR_TWO.y);
    Point prop_pointB_two = new Point(
            BOX_ANCHOR_TWO.x + BOX_WIDTH,
            BOX_ANCHOR_TWO.y + BOX_HEIGHT);

    // Anchor point definitions for 3nd box
    Point prop_pointA_three = new Point(
            BOX_ANCHOR_THREE.x,
            BOX_ANCHOR_THREE.y);
    Point prop_pointB_three = new Point(
            BOX_ANCHOR_THREE.x + BOX_WIDTH,
            BOX_ANCHOR_THREE.y + BOX_HEIGHT);

    // Running variable storing the parking position
    private volatile PropSide PropPos = PropSide.UNKNOWN;
    private volatile double PropPosDistance = 0.0;
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
        propDetectPos(input, RED);

        // detect cone location
        if (!stopConeDetectPipeLine) {
            Logging.log("Start Opcv process to detect cone.");
            conePositionDetect(input);
        }

        // draw a rectangle on sleeve.
        Imgproc.rectangle(
                    input,
                    prop_pointA_one,
                    prop_pointB_one,
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
    public PropSide getPropPos() {
        if (coneDetected == true) {
            PropPos = PropSide.RIGHT;


        }
        else {
            PropPos = PropSide.UNKNOWN;
        }
        return PropPos;
    }

    public double getPropPosDistance() {
        return PropPosDistance;
    }

    // Returns bool indicating whether or not the prop is detected in ANY of the spike tape positions
    public boolean isConeDetected() {
        return coneDetected;
    }

    private void propDetectPos(Mat ImageInput, Scalar Color) {
        Logging.log("Start Opcv process to detect sleeve color.");
        // Select three sections to search for prop of specified color
        Mat areaMat_one = ImageInput.submat(new Rect(prop_pointA_one, prop_pointB_one));
        Mat areaMat_two = ImageInput.submat(new Rect(prop_pointA_two, prop_pointB_two));
        Mat areaMat_three = ImageInput.submat(new Rect(prop_pointA_three, prop_pointB_three));
        Scalar sumColors_one = Core.sumElems(areaMat_one);
        Scalar sumColors_two = Core.sumElems(areaMat_two);
        Scalar sumColors_three = Core.sumElems(areaMat_three);

        //converts matrix into image
        

        // Get the minimum RGB value from every single channel
        double maxColor = Math.max(sumColors_one.val[0], Math.max(sumColors_one.val[1], sumColors_one.val[2]));
        Logging.log("Sleeve max color = %.2f, %.2f, %.2f", sumColors_one.val[0], sumColors_one.val[1], sumColors_one.val[2]);

        // Change the bounding box color based on the sleeve color
        if (maxColor < Math.ulp(0)){
            PropPos = PropSide.UNKNOWN;
        }
        else if (Math.abs(sumColors_one.val[0] - maxColor) < Math.ulp(0)) {
            PropPos = PropSide.LEFT;
            brushColor = RED;
            PropPosDistance = -24;
        } else {
            PropPos = PropSide.RIGHT;
            brushColor = BLUE;
            PropPosDistance = 24;
        }
        Logging.log("Sleeve position: %s", PropPos.toString());

        // Release and return input
        areaMat_one.release();
    }

    private void conePositionDetect(Mat inputCone) {
        // Get the submat frame, and then sum all the values

        Mat areaMat_one = inputCone.submat(new Rect(pointA, pointB));

        Mat doubleareaMat_one = new Mat(areaMat_one.size(), 30); // CV_64FC3, CV_64FC4=30
        areaMat_one.convertTo(doubleareaMat_one, 30);

        int kernelSize = 32;
        Mat destMat = new Mat(doubleareaMat_one.rows(), doubleareaMat_one.cols(), doubleareaMat_one.type());

        double [] filterK = new double[doubleareaMat_one.channels()];
        for (int i = 0; i < doubleareaMat_one.channels(); i++)
        {
            filterK[i] = 1.0/kernelSize/kernelSize;
        }

        Mat kernel = new Mat(kernelSize, kernelSize, doubleareaMat_one.type() ) {
            {
                for (int i = 0; i < kernelSize; i++) {
                    for (int j = 0; j < kernelSize; j++) {
                        put(i, j, filterK);
                    }
                }
            }
        };

        Imgproc.filter2D(doubleareaMat_one, destMat, -1, kernel);

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
    /*
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
        */
        // Release and return input
        areaMat_one.release();
        lineM.release();
        lineMatA.release();
        kernel.release();
        doubleareaMat_one.release();
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
                PropPos = PropPos.LEFT;
                brushColor = RED;
                PropPosDistance = -24.0;
            } else {
                PropPos = PropPos.RIGHT;
                brushColor = BLUE;
                PropPosDistance = 24.0;
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
