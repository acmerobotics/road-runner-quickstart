package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia.blahblah;
import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineFromScratch extends OpenCvPipeline implements VisionProcessor {

//        public PipelineFromScratch.Location location = PipelineFromScratch.location;

    int cameraWidth = 1920;
    int cameraHeight = 1080;
    private OpenCvCamera controlhubcamera;

    boolean isred = false;


    final Point BOTTOMRIGHT = new Point(1, 1);
    final Point BOTTOMLEFT = new Point(1919, 1);
    final Point TOPRIGHT = new Point(1, 1079);
    final Point TOPLEFT = new Point(1919,1079);

    int RegionWidth = cameraWidth/3;
    int RegionHeight = cameraHeight;

    Point region1topright = new Point(
           TOPRIGHT.x-RegionWidth,
            TOPRIGHT.y+0);




    Point region3bottomright = new Point(
            BOTTOMLEFT.x+RegionWidth,
            BOTTOMLEFT.y+0);


    Point region3upperleft = new Point(
            TOPRIGHT.x-RegionHeight,
            TOPRIGHT.y+0);


    public Mat largerect;
    Mat HSvMat = new Mat();



    public void init(int width, int height, CameraCalibration calibration) {

        //submats


        largerect = HSvMat.submat(new Rect(BOTTOMLEFT,TOPRIGHT));
    }

            @Override
        public Mat processFrame(Mat frame) {
            //Imgproc;
            double LowerHueThreshold = 0;
            double UpperHueThreshold = 131;
            double[] avg1;

            Scalar RedHueThreshold ;

            avg1 = new double[]{Core.mean(largerect).val[0]};

            if(avg1[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold){
                isred = true;


            }
            return null;





        }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

//        @Override
//        public void onViewportTapped() {
//            //screen is clicked or tapped
//
//        }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double LowerHueThreshold = 0;
        double UpperHueThreshold = 131;
        double[] avg1;

        avg1 = new double[]{Core.mean(largerect).val[0]};

        if (avg1[0] < UpperHueThreshold && avg1[0] > LowerHueThreshold) {
            isred = true;


            return null;

        }
        return null;


    }
}





