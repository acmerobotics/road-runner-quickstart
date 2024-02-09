package org.firstinspires.ftc.teamcode.drive.opmode.visionCenterStage;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Reconizer extends OpenCvPipeline {
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

        colorMask.copyTo(output);
        return output;
    }
}
