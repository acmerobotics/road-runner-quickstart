package org.firstinspires.ftc.teamcode.Pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class File extends OpenCvPipeline {

    Telemetry telemetry;

    public File(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat mat = new Mat();
    Mat yellowmat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar yellowhighHSV = new Scalar(248, 200, 250);
        Scalar yellowlowHSV = new Scalar(138, 34, 143);
        Rect RECT_LEFT = new Rect(
                new Point(180, 30),
                new Point(240,100)
        );
        Rect RECT_MIDDLE = new Rect(
                new Point(130, 160),
                new Point(180,110)
        );
        /*Rect RECT_RIGHT = new Rect(
                new Point(220, 160),
               new Point(270, 110)
        );*/
        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, RECT_LEFT, color, 2);
        Imgproc.rectangle(mat, RECT_MIDDLE, color, 2);
        //Imgproc.rectangle(mat, RECT_RIGHT, color, 2);
        Core.inRange(mat, yellowlowHSV, yellowhighHSV, yellowmat);
        Mat left = yellowmat.submat(RECT_LEFT);
        Mat center = yellowmat.submat(RECT_MIDDLE);
        //Mat right = yellowmat.submat(RECT_RIGHT);
        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(center).val[0];
        //double rightValue = Core.sumElems(right).val[0];

        /*if (leftValue > middleValue && leftValue > rightValue) {
            telemetry.addData("Duck", "Left");
        }
        if (middleValue > leftValue && middleValue > rightValue) {
            telemetry.addData("Duck", "Middle");
        }
        if (rightValue > middleValue && rightValue > leftValue) {
            telemetry.addData("Duck", "Right");
        }*/

        telemetry.addData("leftValue", leftValue);
        telemetry.addData("middleValue", middleValue);
        //telemetry.addData("rightValue", rightValue);
        telemetry.update();
        return mat;
    }

}
