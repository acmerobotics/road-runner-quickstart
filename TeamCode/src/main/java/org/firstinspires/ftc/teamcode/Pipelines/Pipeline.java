package org.firstinspires.ftc.teamcode.Pipelines;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public Pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat mat = new Mat();
    Mat yellowmat = new Mat();
    Mat redmat = new Mat();
    Mat redmat2 = new Mat();
    Mat bluemat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar yellowHighHSV = new Scalar(63, 100, 75);//3 numbers to represent something, in this case, the rgb values of yellowhigh
        Scalar yellowLowHSV = new Scalar(41, 34, 75);
        Scalar redHighHSV1 = new Scalar (359, 100, 75);
        Scalar redLowHSV1 = new Scalar (346, 100, 75);
        Scalar redHighHSV2 = new Scalar (10, 100, 75);
        Scalar redLowHSV2 = new Scalar (0, 100, 75);
        Scalar blueHighHSV = new Scalar (256, 100, 75);
        Scalar blueLowHSV = new Scalar (210, 100, 75);

        Rect RECT_MIDDLE = new Rect(200, 200, 200, 200);

        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, RECT_MIDDLE, color, 2);



        Core.inRange(mat, yellowLowHSV, yellowHighHSV, yellowmat);//core.inRange = if color of a pixel is in color range, then it selects it and colors it white, otherwise it colors it black
        Core.inRange(mat, redLowHSV1, redHighHSV1, redmat);
        Core.inRange(mat, redLowHSV2, redHighHSV2, redmat2);
        Core.inRange(mat, blueLowHSV, blueHighHSV, bluemat);


        Core.bitwise_or(redmat, redmat2, redmat); //combines both redmats and stores them into redmat

        Mat yellowcenter = yellowmat.submat(RECT_MIDDLE);
        Mat redcenter = redmat.submat(RECT_MIDDLE);
        Mat bluecenter = bluemat.submat(RECT_MIDDLE);

        double middleyellowValue = Core.sumElems(yellowcenter).val[0];//val is value
        double middleredValue = Core.sumElems(redcenter).val[0];//val is value
        double middleblueValue = Core.sumElems(bluecenter).val[0];//val is value

        double minNum = 0.5;
        double rectArea = RECT_MIDDLE.area();

        if(middleyellowValue /rectArea > minNum){
           telemetry.addData("Yellow Detected", middleyellowValue) ;
        }
        else if (middleblueValue/rectArea>minNum){
           telemetry.addData("Blue Detected", middleblueValue);
        }
        else if (middleredValue/rectArea>minNum){
            telemetry.addData("Red Detected", middleredValue);
        }
        else {
            telemetry.addData("Not Detected", 0);
        }

        //telemetry.addData("leftValue", leftValue);
        //telemetry.addData("middleValue", middleValue);
        //telemetry.addData("rightValue", rightValue);
        telemetry.update();
        return mat;
    }

}
