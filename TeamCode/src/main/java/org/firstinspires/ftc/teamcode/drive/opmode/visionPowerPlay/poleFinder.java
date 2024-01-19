package org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class poleFinder extends OpenCvPipeline {
    public enum poleLocation { // Set up an enum to describe pole location
        LEFT,
        RIGHT,
        ALIGNED
    }

    private final int width;
    private final int height;
    private final int slices;  // Declare variables to hold frame size and number of slices
    poleLocation outputLocation;        // Declare variable to hold pole location information

    // YcBcR will hold mat with converted colors
    // output will be returned in the end
    // mask will hold mask information
    Mat YcBcR = new Mat(), output = new Mat(), mask = new Mat();

    // Set up variables related to frame slicing, but wait until constructor to initialize
    boolean[] isYellow;
    Mat[] verticalMat;
    Rect[] verticalRects;
    Scalar[] verticalAvg;

    // Declare scalars that represent our color thresholds
    Scalar lower = new Scalar(0.0, 110.0, 27.0);
    Scalar upper = new Scalar(255.0, 195.0, 100.0);

    // Define colors for screen elements
    Scalar rectBackground = new Scalar(255.0, 0.0, 0.0);
    Scalar rectSubject = new Scalar(0.0, 255.0, 0.0);
    Scalar textColor = new Scalar(0.0, 0.0, 255.0);

    public poleFinder() {   // Create a default constructor to initialize certain variables
        this.width = 1280;
        this.height = 720;
        this.slices = 64;

        isYellow = new boolean[this.slices];
        verticalMat = new Mat[this.slices];
        verticalRects = new Rect[this.slices];
        verticalAvg = new Scalar[this.slices];
    }

    public poleFinder(int width, int height, int slices) {  // Create a constructor that allows user to specify certain variables
        this.width = width;
        this.height = height;
        this.slices = slices;

        isYellow = new boolean[slices];
        verticalMat = new Mat[slices];
        verticalRects = new Rect[slices];
        verticalAvg = new Scalar[slices];
    }

    @Override
    public Mat processFrame(Mat input) {

        // Convert input to the YcBcR color space
        // This is more reliable for seeing game elements in different lighting conditions
        Imgproc.cvtColor(input, YcBcR, Imgproc.COLOR_RGB2YCrCb);

        // Copy the unedited input to output to aid in debugging
        input.copyTo(output);

        // Create a mask of the yellow in the image based on our threshold values
        Core.inRange(YcBcR, lower, upper, mask);

        // Create a temporary boolean array so that we don't have conflicts with reading the main
        boolean[] tempYellow = new boolean[slices];

        // This loop operates on each slice of the image
        for (int currentSlice = 0; currentSlice < slices; currentSlice++) {
            // Generate rectangle based on screen resolution and number of slices
            verticalRects[currentSlice] = new Rect((currentSlice * (width / slices)), 1, (width / slices), (height - 1));
            // Create submat of mask from the rectangle
            verticalMat[currentSlice] = mask.submat(verticalRects[currentSlice]);
            // Represent each submat as a number from 0-255 based on how yellow it is
            verticalAvg[currentSlice] = Core.mean(verticalMat[currentSlice]);

            // Check if a certain amount of the current slice is yellow
            if (verticalAvg[currentSlice].val[0] > 196) {
                // If we deem the slice yellow, we draw a rectangle with the subject color and set
                // a flag to true
                Imgproc.rectangle(output, verticalRects[currentSlice], rectSubject, 2);
                tempYellow[currentSlice] = true;
            } else {
                // If we deem the slice not yellow, we draw a rectangle with the backround color and
                // a flag to false
                Imgproc.rectangle(output, verticalRects[currentSlice], rectBackground, 2);
                tempYellow[currentSlice] = false;
            }

            // Clone the temporary array to the main array
            isYellow = tempYellow.clone();
        }

        if (getLocation() == poleLocation.LEFT) {
            Imgproc.putText(output, "LEFT", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
        } else if (getLocation() == poleLocation.RIGHT) {
            Imgproc.putText(output, "RIGHT", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
        } else if (getLocation() == poleLocation.ALIGNED) {
            Imgproc.putText(output, "ALIGNED", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
        }

        return output;
    }

    public poleLocation getLocation() {
        int totalYellow = 0;
        int targetIndex = slices / 2;
        int left = 0, right = 0;

        for (int i = 0; i < slices; i++) {
            if (isYellow[i]) {
                totalYellow++;
            }

        }

        if (totalYellow == 0) {
            return poleLocation.ALIGNED;
        }

        if (isYellow[targetIndex - 2] && isYellow[targetIndex - 1] && isYellow[targetIndex] && isYellow[targetIndex + 1]) {
            return poleLocation.ALIGNED;
        }

        for (int i = 0; i < slices; i++) {
            if (isYellow[i]) {
                if (i < targetIndex - 1) {
                    left ++;
                } else {
                    right ++;
                }
            }
        }

        if (left > right) {
            return poleLocation.LEFT;
        } else {
            return  poleLocation.RIGHT;
        }
    }

}
