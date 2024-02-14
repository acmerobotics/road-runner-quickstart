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
    }
    private final Mat YcBcR = new Mat();
    private Mat leftMat = new Mat();
    private Mat middleMat = new Mat();
    private Mat rightMat = new Mat();

    private double leftAvg, middleAvg, rightAvg;
    private final Mat output = new Mat();
    private final Rect leftRect = new Rect(0,0,440,720);
    private final Rect middleRect = new Rect(440,0,420,720);
    private final Rect rightRect = new Rect(860,0,420,720);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input,YcBcR, Imgproc.COLOR_RGB2YCrCb);

        leftMat = input.submat(leftRect);
        Core.inRange(YcBcR, new Scalar(16.0, 0.0, 0.0), new Scalar(230, 128.0, 255.0), leftMat);

        leftAvg = Core.mean(leftMat).val[0]*100000.0;

        middleMat = input.submat(middleRect);
        Core.inRange(YcBcR, new Scalar(16.0, 0.0, 0.0), new Scalar(230, 128.0, 255.0), middleMat);

        middleAvg = Core.mean(middleMat).val[0]*100000.0;

        rightMat = input.submat(rightRect);
        Core.inRange(YcBcR, new Scalar(16.0, 0.0, 0.0), new Scalar(230, 128.0, 255.0), rightMat);

        rightAvg = Core.mean(rightMat).val[0]*100000.0;

        input.copyTo(output);

        Imgproc.rectangle(output, leftRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, middleRect, new Scalar(255.0, 0.0, 0.0), 2);
        Imgproc.rectangle(output, rightRect, new Scalar(255.0, 0.0, 0.0), 2);

        Imgproc.putText(output, getPixelLocation().name(), new Point(25,100), Imgproc.FONT_HERSHEY_SIMPLEX,3.0,new Scalar(0.0, 255.0, 0.0));

        return output;
    }
    public pixelLocation getPixelLocation() {
        if (leftAvg > middleAvg && leftAvg > rightAvg) {
            return pixelLocation.LEFT;
        } else if (rightAvg > leftAvg && rightAvg > middleAvg) {
            return pixelLocation.RIGHT;
        } else {
            return pixelLocation.MIDDLE;
        }
    }
}
