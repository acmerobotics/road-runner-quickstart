package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends OpenCvPipeline {

    // percentage color threshold
    private static final double percentColorThreshold = 0.1;

    // green signal sleeve hsv values
    private static final Scalar lowGreenSignalHSV = new Scalar(143, 255, 84);
    private static final Scalar highGreenSignalHSV = new Scalar(143, 255, 255);
    //pink signal sleeve hsv values
    private static final Scalar lowPinkSignalHSV = new Scalar(307, 255, 204);
    private static final Scalar highPinkSignalHSV = new Scalar(307, 255, 255);
    //purple signal sleeve hsv values
    private static final Scalar lowPurpleSignalHSV = new Scalar(282, 255, 153);
    private static final Scalar highPurpleSignalHSV = new Scalar(282, 255, 255);

    private final Telemetry telemetry;

    private SleeveDirection sleeveDirection;

    public Vision(final Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public enum SleeveDirection {
        LEFT,
        CENTER,
        RIGHT
    }

    public SleeveDirection getSleeveDirection() {
        return sleeveDirection;
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

        final Mat greenMatrix = new Mat();
        final Mat pinkMatrix = new Mat();
        final Mat purpleMatrix = new Mat();

        Core.inRange(lowerHalfHSV, lowGreenSignalHSV, highGreenSignalHSV, greenMatrix);
        Core.inRange(lowerHalfHSV, lowPinkSignalHSV, highPinkSignalHSV, pinkMatrix);
        Core.inRange(lowerHalfHSV, lowPurpleSignalHSV, highPurpleSignalHSV, purpleMatrix);

        final double greenPercentage = Core.sumElems(greenMatrix).val[0] / HALF_RECT.area() / 255;
        final double pinkPercentage = Core.sumElems(pinkMatrix).val[0] / HALF_RECT.area() / 255;
        final double purplePercentage = Core.sumElems(purpleMatrix).val[0] / HALF_RECT.area() / 255;

        final double maxPercentage = Math.max(Math.max(greenPercentage, pinkPercentage), purplePercentage);
        if (maxPercentage < percentColorThreshold)
            return inputMatrix;
        else {
            if (Double.compare(greenPercentage, maxPercentage) == 0)
                sleeveDirection = SleeveDirection.LEFT;
            else if (Double.compare(pinkPercentage, maxPercentage) == 0)
                sleeveDirection = SleeveDirection.CENTER;
            else
                sleeveDirection = SleeveDirection.RIGHT;
        }

        telemetry.addData("SleeveDirection", (Object)sleeveDirection);
        telemetry.update();

        return inputMatrix;

    }

}

