package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends OpenCvPipeline {

    // green signal sleeve hsv values
    private static final Scalar lowGreenSignalHSV = new Scalar(23, 50, 70);
    private static final Scalar highGreenSignalHSV = new Scalar(32, 255, 255);
    //pink signal sleeve hsv values
    private static final Scalar lowPinkSignalHSV = new Scalar(23, 50, 70);
    private static final Scalar highPinkSignalHSV = new Scalar(32, 255, 255);
    //purple signal sleeve hsv values
    private static final Scalar lowPurpleSignalHSV = new Scalar(23, 50, 70);
    private static final Scalar highPurpleSignalHSV = new Scalar(32, 255, 255);



    private final Telemetry telemetry;

    public Vision(final Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(final Mat inputMatrix) {

        //dividing the input by half to increase efficiency
        final int rows = inputMatrix.rows();
        final int cols = inputMatrix.cols();

        final Rect HALF_RECT = new Rect(
                new Point(0, rows / 2d),
                new Point(cols, rows)
        );

        //setting the half screen as ROI
        final Mat lowerHalfMatrix = inputMatrix.submat(HALF_RECT);
        final Mat lowerHalfHSV = new Mat();

        //returning a grayscale image of what matches hsv ranges
        Imgproc.cvtColor(lowerHalfMatrix, lowerHalfHSV, Imgproc.COLOR_RGB2HSV);



        return null;

    }

}

