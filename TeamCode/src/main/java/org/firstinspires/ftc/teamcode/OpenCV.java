package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCV {
    Telemetry telemetry;
    Mat mat = new Mat();
    public OpenCV(Telemetry t) { telemetry = t;}

//    @Override
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Scalar lowHSV = new Scalar(23, 50, 70);
//        Scalar highHSV = new Scalar(32, 255, 255);
//
//       Core.inRange(mat, lowHSV, highHSV, mat);
//
//
//    }
}
