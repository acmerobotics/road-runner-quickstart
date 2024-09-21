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
        Scalar yellowhighHSV = new Scalar(248, 200, 250);//3 numbers to represent something, in this case, the rgb values of yellowhigh
        Scalar yellowlowHSV = new Scalar(138, 34, 143);//WHY DOES EVERY HSV CALAULATOR HAVE DIFFERENT VALUES OPRIUHGPDAUFHMPGI RDGPID
        Scalar redHighHSV1 = new Scalar (359, 255, 191);
        Scalar redLowHSV1 = new Scalar (346, 255, 191);
        Scalar redHighHSV2 = new Scalar (10, 255, 191); 
        Scalar redLowHSV2 = new Scalar (0, 255, 191);
        Scalar blueHighHSV = new Scalar (256, 255, 191);
        Scalar blueLowHSV = new Scalar (210, 255, 191);
        /*Rect RECT_LEFT = new Rect(
                new Point(180, 30),//top left of camera fov is (0,0), pos x is downward, pos y is rightward
                new Point(240,100)//max values is 720x by 480y
        );*/
        Rect RECT_MIDDLE2 = new Rect(
                new Point ( 100, 25),
                new Point(650, 400)
        );
        /*Rect RECT_MIDDLE = new Rect(
                new Point(130, 160),
                new Point(180,110)
        );
        */

        /*Rect RECT_RIGHT = new Rect(
                new Point(220, 160),
               new Point(270, 110)
        );*/
        Scalar color = new Scalar(64, 64, 64);
       // Imgproc.rectangle(mat, RECT_LEFT, color, 2);//mat is array with 3 layers for each of the r,g,b values of each pixel
        Imgproc.rectangle(mat, RECT_MIDDLE2, color, 2);//Imgproc does something idk
        //Imgproc.rectangle(mat, RECT_RIGHT, color, 2);
        Core.inRange(mat, yellowlowHSV, yellowhighHSV, yellowmat);//core.inRange = if color of a pixel is in color range, then it selects it and colors it white, otherwise it colors it black
        Core.inRange(mat, redLowHSV1, redHighHSV1, redmat);
        Core.inRange(mat, redLowHSV2, redHighHSV2, redmat2);
        Core.inRange(mat, blueLowHSV, blueHighHSV, bluemat);
        // Mat left = yellowmat.submat(RECT_LEFT);
        Mat yellowcenter = yellowmat.submat(RECT_MIDDLE2);
        Mat redcenter = redmat.submat(RECT_MIDDLE2);
        Mat redcenter2 = redmat2.submat(RECT_MIDDLE2);
        Mat bluecenter = bluemat.submat(RECT_MIDDLE2);
        Core.bitwise_or(redmat, redmat2, redmat); //combines both redmats and stores them into redmat
        //Mat right = yellowmat.submat(RECT_RIGHT);
     //   double leftValue = Core.sumElems(left).val[0];
        double middleyellowValue = Core.sumElems(yellowcenter).val[0];//val is value
        double middleredValue = Core.sumElems(redcenter).val[0];//val is value
        double middleblueValue = Core.sumElems(bluecenter).val[0];//val is value
        //double rightValue = Core.sumElems(right).val[0];
        double minNum = 0.5;
        double rectArea = RECT_MIDDLE2.area();
        if(middleyellowValue/rectArea>minNum){
           telemetry.addData("Yellow Detected", 1) ;
        }
        else if (middleblueValue/rectArea>minNum){
           telemetry.addData("Blue Detected", 1);
        }
        else if (middleredValue/rectArea>minNum){
            telemetry.addData("Red Detected", 1);
        }
        else {
            telemetry.addData("Not Detected", 0);
        }


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
