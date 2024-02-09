package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Recognizer extends OpenCvPipeline {
    public enum pixelLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }
    public pixelLocation Location = pixelLocation.UNKNOWN;
    public final Scalar blueLower = new Scalar(0.0, 0.0, 0.0);
    public final Scalar blueUpper = new Scalar(255.0,255.0,255.0);

    private final Mat YcBcR = new Mat();
    private final Mat colorMask = new Mat();
    private final Mat leftMat = new Mat();
    private final Mat middleMat = new Mat();
    private final Mat rightMat = new Mat();
    private final Mat unknownMat = new Mat();

    private double leftAvg, middleAvg, rightAvg;
    private final Mat output = new Mat();
    private final Rect leftRect = new Rect(0,0,420,720);
    private final Rect middleRect = new Rect(420,0,440,720);
    private final Rect rightRect = new Rect(860,0,420,720);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YcBcR,Imgproc.COLOR_RGB2YCrCb);

        Mat colorMask = new Mat();
        Core.inRange(YcBcR, blueLower, blueUpper, colorMask);

        colorMask.copyTo(leftMat);
        leftMat.submat(leftRect);

        leftAvg = Core.mean(leftMat).val[0];

        colorMask.copyTo(middleMat);
        middleMat.submat(middleRect);

        middleAvg = Core.mean(middleMat).val[0];

        colorMask.copyTo(rightMat);
        rightMat.submat(rightRect);

        rightAvg = Core.mean(rightMat).val[0];

        input.copyTo(output, colorMask);

        Imgproc.rectangle(output, leftRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, middleRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, rightRect, new Scalar(255.0, 0.0, 0.0), 2);

        Imgproc.putText(output, getPixelLocation().name(), new Point(25,100), Imgproc.FONT_HERSHEY_SIMPLEX,3.0,new Scalar(0.0, 255.0, 0.0));

        return output;
    }
    public pixelLocation getPixelLocation() {
        if (leftAvg > middleAvg && leftAvg > rightAvg) {
            return pixelLocation.LEFT;
        } else if (middleAvg > leftAvg && middleAvg > rightAvg) {
            return pixelLocation.MIDDLE;
        } else if (rightAvg > leftAvg && rightAvg > middleAvg) {
            return pixelLocation.RIGHT;
        } else {
            return pixelLocation.UNKNOWN;
        }
    }
}
