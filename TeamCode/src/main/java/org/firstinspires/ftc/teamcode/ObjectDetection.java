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
 * Used to take 3 submats of image(1 for left, 2 for center, and 3 for right) and find/locate the prop
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

    public enum ColorS {
        RED,
        BLUE
    }

    // TOPLEFT anchor point for the 3 bounding boxes
    private final Point BOX_ANCHOR_ONE = new Point(28, 124);
    private final Point BOX_ANCHOR_TWO = new Point(158, 116);
    private final Point BOX_ANCHOR_THREE = new Point(288, 124);

    // Width and height for the bounding boxes
    private final int BOX_WIDTH = 4;
    private final int BOX_HEIGHT = 6;

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
    public volatile double hChannelAve = 0;

    // TOP-LEFT anchor point for the bounding box
    public Point Object_TOPLEFT_POINT = new Point(0, 0);

    // Width and height for the bounding box
    public int REGION_WIDTH = 320;
    public int REGION_HEIGHT = 240;

    // Color definitions
    private final Scalar
            RED   = new Scalar(255, 0, 0),
            GREY  = new Scalar(0, 0, 255);
    private Scalar rectColor1 = new Scalar(0, 0, 0);
    private Scalar rectColor2 = new Scalar(0, 0, 0);
    private Scalar rectColor3 = new Scalar(0, 0, 0);

    // Anchor point definitions
    Point pointA = new Point(
            Object_TOPLEFT_POINT.x,
            Object_TOPLEFT_POINT.y);
    Point pointB = new Point(
            Object_TOPLEFT_POINT.x + REGION_WIDTH,
            Object_TOPLEFT_POINT.y + REGION_HEIGHT);

    private ColorS detectColor = ColorS.RED;

    public void setColorFlag(ColorS nColor) {
        detectColor = nColor;
    }


    @Override
    public Mat processFrame(Mat input) {
        // detect sleeve color.
        propDetectPos(input, detectColor);

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
                    rectColor1,
                    2
        );
        Imgproc.rectangle(
                input,
                prop_pointA_two,
                prop_pointB_two,
                rectColor2,
                2
        );
        Imgproc.rectangle(
                input,
                prop_pointA_three,
                prop_pointB_three,
                rectColor3,
                2
        );

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public PropSide getPropPos() {
        /*if (coneDetected == true) {
            PropPos = PropSide.RIGHT;
        }
        else {
            PropPos = PropSide.UNKNOWN;
        }
        */return PropPos;
    }

    public double getPropPosDistance() {
        return PropPosDistance;
    }


    private void propDetectPos(Mat ImageInput, ColorS Color) {
        Logging.log("Start Opcv process to detect sleeve color.");
        // Select three sections to search for prop of specified color
        Mat areaMat_one = ImageInput.submat(new Rect(prop_pointA_one, prop_pointB_one));
        Mat areaMat_two = ImageInput.submat(new Rect(prop_pointA_two, prop_pointB_two));
        Mat areaMat_three = ImageInput.submat(new Rect(prop_pointA_three, prop_pointB_three));
        Scalar sumColors_one = Core.sumElems(areaMat_one);
        Scalar sumColors_two = Core.sumElems(areaMat_two);
        Scalar sumColors_three = Core.sumElems(areaMat_three);

        int colorChannelIndex = 0;
        if (ColorS.RED == Color) {
            colorChannelIndex = 0; // red
        } else {
            colorChannelIndex = 2; // blue
        }

        // Boolean to see if a box is containing real colors or not(identifies between noise and actual red)
        boolean boxNoise1 = false;
        boolean boxNoise2 = false;
        boolean boxNoise3 = false;

        Logging.log("Box 1 RGB = %.2f, %.2f, %.2f", sumColors_one.val[0], sumColors_one.val[1], sumColors_one.val[2]);
        Logging.log("Box 2 RGB = %.2f, %.2f, %.2f", sumColors_two.val[0], sumColors_two.val[1], sumColors_two.val[2]);
        Logging.log("Box 3 RGB = %.2f, %.2f, %.2f", sumColors_three.val[0], sumColors_three.val[1], sumColors_three.val[2]);

        // Get the average of the RGB Values from each box
        double avgColor1 = (sumColors_one.val[0] + sumColors_one.val[1] + sumColors_one.val[2]) / 3;
        double avgColor2 = (sumColors_two.val[0] + sumColors_two.val[1] + sumColors_two.val[2]) / 3;
        double avgColor3 = (sumColors_three.val[0] + sumColors_three.val[1] + sumColors_three.val[2]) / 3;
        Logging.log("Box 1 mean = %.2f", avgColor1);
        Logging.log("Box 2 mean = %.2f", avgColor2);
        Logging.log("Box 3 mean = %.2f", avgColor3);
        
        // Find the standard deviation for the red value of every box
        double stColor1 = sumColors_one.val[colorChannelIndex] / avgColor1;
        double stColor2 = sumColors_two.val[colorChannelIndex] / avgColor2;
        double stColor3 = sumColors_three.val[colorChannelIndex] / avgColor3;
        Logging.log("Box 1 red/blue st.deviation = %.2f", stColor1);
        Logging.log("Box 2 red/blue st.deviation = %.2f", stColor2);
        Logging.log("Box 3 red/blue st.deviation = %.2f", stColor3);
        
        // Differentiate between noise and color data
        if (stColor1 >= 1.4) {
            boxNoise1 = true;
            rectColor1 = RED;
        }
        if (stColor2 >= 1.4) {
            boxNoise2 = true;
            rectColor2 = RED;
        }
        if (stColor3 >= 1.4) {
            boxNoise3 = true;
            rectColor3 = RED;
        }

        // Log for noise
        Logging.log("Box 1 obj detected: %d", boxNoise1?1:0);
        Logging.log("Box 2 obj detected: %d", boxNoise2?1:0);
        Logging.log("Box 3 obj detected: %d", boxNoise3?1:0);

        // Change the prop location info based on standard deviation
        if (stColor1 > stColor2 && stColor1 > stColor3) {
            PropPos = PropSide.LEFT;
        } else if (stColor2 > stColor1 && stColor2 > stColor3) {
            PropPos = PropSide.CENTER;
        } else if (stColor3 > stColor1 && stColor3 > stColor2) {
            PropPos = PropSide.RIGHT;
        } else {
            PropPos = PropSide.UNKNOWN;
        }

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
