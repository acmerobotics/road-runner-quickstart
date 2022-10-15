package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;

public class ConeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();


    public ConeDetector(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 75);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

//        static final Rect LEFT_ROI = new Rect() ();
//
//        static final Rect RIGHT_ROI = new Rect() ();

//        Mat left = mat.submat(LEFT_ROI);
//        Mat right = mat.submat(RIGHT_ROI);


//        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 240;
//        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 240;
//
//        left.release();
//        right.release();

//        telemetry.addData(caption"", (int) Core.sumElems(left).val[0]);
//        telemetry.addData(caption"", (int) Core.sumElems(right).val[0]);
//        telemetry.addData(caption"", value:Math.round(leftValue * 100) + "%");
//        telemetry.addData(caption"", value:Math.round(rightValue * 100) + "%");

        return null;
    }
}