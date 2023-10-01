package org.firstinspires.ftc.teamcode;

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

    private ColorS detectColor = ColorS.RED;

    public void setColorFlag(ColorS nColor) {
        detectColor = nColor;
    }

    private boolean debug_flag = false;

    @Override
    public Mat processFrame(Mat input) {
        // detect sleeve color.
        propDetectPos(input, detectColor);

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
        return PropPos;
    }

    private void propDetectPos(Mat ImageInput, ColorS Color) {
        if (debug_flag) {
            Logging.log("Start OpCV process to detect prop location.");
        }

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

        if (debug_flag) {
            Logging.log("Box 1 RGB = %.2f, %.2f, %.2f", sumColors_one.val[0], sumColors_one.val[1], sumColors_one.val[2]);
            Logging.log("Box 2 RGB = %.2f, %.2f, %.2f", sumColors_two.val[0], sumColors_two.val[1], sumColors_two.val[2]);
            Logging.log("Box 3 RGB = %.2f, %.2f, %.2f", sumColors_three.val[0], sumColors_three.val[1], sumColors_three.val[2]);
        }
        // Get the average of the RGB Values from each box
        double avgColor1 = (sumColors_one.val[0] + sumColors_one.val[1] + sumColors_one.val[2]) / 3;
        double avgColor2 = (sumColors_two.val[0] + sumColors_two.val[1] + sumColors_two.val[2]) / 3;
        double avgColor3 = (sumColors_three.val[0] + sumColors_three.val[1] + sumColors_three.val[2]) / 3;

        if (debug_flag) {
            Logging.log("Box 1 mean = %.2f", avgColor1);
            Logging.log("Box 2 mean = %.2f", avgColor2);
            Logging.log("Box 3 mean = %.2f", avgColor3);
        }
        // Find the standard deviation for the red value of every box
        double stColor1 = sumColors_one.val[colorChannelIndex] / avgColor1;
        double stColor2 = sumColors_two.val[colorChannelIndex] / avgColor2;
        double stColor3 = sumColors_three.val[colorChannelIndex] / avgColor3;

        if (debug_flag) {
            Logging.log("Box 1 red/blue st.deviation = %.2f", stColor1);
            Logging.log("Box 2 red/blue st.deviation = %.2f", stColor2);
            Logging.log("Box 3 red/blue st.deviation = %.2f", stColor3);
        }

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
        if (debug_flag) {
            Logging.log("Box 1 obj detected: %d", boxNoise1 ? 1 : 0);
            Logging.log("Box 2 obj detected: %d", boxNoise2 ? 1 : 0);
            Logging.log("Box 3 obj detected: %d", boxNoise3 ? 1 : 0);
        }
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
}
