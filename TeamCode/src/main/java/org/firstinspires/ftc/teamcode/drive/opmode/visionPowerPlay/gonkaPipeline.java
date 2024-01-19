package org.firstinspires.ftc.teamcode.drive.opmode.visionPowerPlay;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class gonkaPipeline extends OpenCvPipeline {
    public enum wiggleDirection {
        LEFT,
        RIGHT,
        STOP
    }


    wiggleDirection outputDirection = wiggleDirection.STOP;

    // Define mats to be used throughout the pipeline
    Mat YcBcr = new Mat();  // Used to store the converted mat as YcBcR
    Mat output = new Mat(); // Mat to be returned at the end of the pipeline
    Mat mask = new Mat();   // Mat that will hold the color-based mask of the scene

    // Define variables related to the slicing of the frame
    // TODO: make this dynamic based on a given resolution and number of slices
    boolean[] IsYellow = new boolean[20];
    Mat[] verticalMat = new Mat[20];
    Rect[] verticalSlices = new Rect[20];
    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
    Scalar[] verticalAvg = new Scalar[20];
    Scalar lower = new Scalar(0.0, 138.8, 51.0);
    Scalar upper = new Scalar(255.0, 178.5, 94.9);

    int left, right;

    @Override
    public Mat processFrame(Mat input) {
        left = 0;
        right = 0;

        Imgproc.cvtColor(input, YcBcr, Imgproc.COLOR_RGB2YCrCb);

        Core.inRange(YcBcr, lower, upper, mask);

        input.copyTo(output);

        for (int currentRect = 0; currentRect < 20; currentRect++) {
            verticalSlices[currentRect] = new Rect((currentRect * 64), 0, 64, 720);
            verticalMat[currentRect] = mask.submat(verticalSlices[currentRect]);
            verticalAvg[currentRect] = Core.mean(verticalMat[currentRect]);
            if (verticalAvg[currentRect].val[0] > 128) {
                Imgproc.rectangle(output, verticalSlices[currentRect], new Scalar(0.0, 255.0, 0.0), 2);
                IsYellow[currentRect] = true;
                if (currentRect < 10) {
                    left++;
                } else {
                    right++;
                }
            } else {
                Imgproc.rectangle(output, verticalSlices[currentRect], rectColor, 2);
                IsYellow[currentRect] = false;
            }
            if (IsYellow[9] && IsYellow[10]) {
                Imgproc.putText(output, "Perfect", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
                outputDirection = wiggleDirection.STOP;
            }
            else
            {
                if (left < right) {
                    Imgproc.putText(output, "Move right", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
                    outputDirection = wiggleDirection.RIGHT;
                }
                else if (right < left) {
                    Imgproc.putText(output, "Move left", new Point(25, 100), Imgproc.FONT_HERSHEY_SIMPLEX, 3.0, new Scalar(0.0, 0.0, 255.0));
                    outputDirection = wiggleDirection.LEFT;
                }
            }
        }

        // Basically i'll put up a mask that only lets yellow through, then it will check to see which square has the most visible (non-masked) pixels to see where the pole would be

        //Imgproc.putText(output, "test text", new Point(25, 100), Imgproc.FONT_HERSHEY_COMPLEX, 2.0, new Scalar(255.0, 0.0, 0.0));

        return output;

    }

    public wiggleDirection getDirection() {
        return this.outputDirection;
    }
}
