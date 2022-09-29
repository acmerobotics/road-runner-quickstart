package org.firstinspires.ftc.teamcode.visionPiplines;


import android.os.Build;

import androidx.annotation.RequiresApi;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
//import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;


public class DetectionPipeline extends OpenCvPipeline {
    private static int coneAreaMin = 300;
    private static int signalAreaMin = 300;

    private static Double signalMin = .06;
    private static Double signalMax = .075;

    private static Double coneMin = .042;
    private static Double coneMax = .05999;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static List<cone> filterCones(Color color, List<MatOfPoint> coneContours, double inputWidth, double inputHeight) {
        List<MatOfPoint> matches = coneContours.stream()
                                            .filter(contour -> Imgproc.contourArea(contour) >= coneAreaMin && Imgproc.contourArea(contour) / Math.pow(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true),2) <= coneMax && Imgproc.contourArea(contour) / Math.pow(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true),2) >= coneMin)
                                            .collect(Collectors.toList());
        List<cone> cones = new ArrayList<cone>();
        for (int i = 0; i < matches.size(); i++) {
            MatOfPoint match = matches.get(i);
            Rect rect = Imgproc.boundingRect(match);
            double area = Imgproc.contourArea(match);
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(match.toArray()),true);
            double ratio = area / Math.pow(perimeter,2);
            double coneX = rect.x + (rect.width / 2.0) - (inputWidth / 2);
            double coneY = -1 * (rect.y + (rect.height / 2.0) - (inputHeight / 2));
            cones.add(new cone(color, coneX, coneY, area, perimeter, ratio, match));
        }
        return cones;
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static List<signal> filterSignals(Color color, List<MatOfPoint> sigContours, double inputWidth, double inputHeight) {
        List<MatOfPoint> matches = sigContours.stream()
                                            .filter(contour -> Imgproc.contourArea(contour) >= signalAreaMin && Imgproc.contourArea(contour) / Math.pow(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true),2) <= signalMax && Imgproc.contourArea(contour) / Math.pow(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true),2) >= signalMin)
                                            .collect(Collectors.toList());
        List<signal> signals = new ArrayList<signal>();
        for (int i = 0; i < matches.size(); i++) {
            MatOfPoint match = matches.get(i);
            Rect rect = Imgproc.boundingRect(match);
            double area = Imgproc.contourArea(match);
            double perimeter = Imgproc.arcLength(new MatOfPoint2f(match.toArray()),true);
            double ratio = area / Math.pow(perimeter,2);
            double signalX = rect.x + (rect.width / 2.0) - (inputWidth / 2);
            double signalY = -1 * (rect.y + (rect.height / 2.0) - (inputHeight / 2));
            signals.add(new signal(color, signalX, signalY, area, perimeter, ratio));
        }
        return signals;
    }

//    Telemetry telemetry;
    public DetectionPipeline() {
    }

    public static List<cone> conesRed = new ArrayList<cone>();
    public static List<cone> conesBlue= new ArrayList<cone>();

    public static List<signal> signalsRed = new ArrayList<signal>();
    public static List<signal> signalsBlue= new ArrayList<signal>();
    public static List<signal> signalsGreen = new ArrayList<signal>();

    // public static Scalar lowerred = new Scalar(16, 163, 95);
    // public static Scalar upperred = new Scalar(156, 242, 130);
    // public static Scalar lowerred = new Scalar(0, 132, 73);
    // public static Scalar upperred = new Scalar(255, 255, 191);
    private static Scalar lowerred = new Scalar(0, 132, 73);
    private static Scalar upperred = new Scalar(255, 255, 110);

    // public static Scalar lowerblue = new Scalar(100, 0, 0);
    // public static Scalar upperblue = new Scalar(150, 255, 255);
    private static Scalar lowerblue = new Scalar(0, 92, 148);
    private static Scalar upperblue = new Scalar(255, 128, 214);

    private static Scalar lowerredalt = new Scalar(0, 160, 143);
    private static Scalar upperredalt = new Scalar(255, 212, 198);

    // public static Scalar lowerbluealt = new Scalar(0, 100, 45);
    // public static Scalar upperbluealt = new Scalar(255, 180, 115);
    private static Scalar lowerbluealt = new Scalar(0, 100, 45);
    private static Scalar upperbluealt = new Scalar(255, 153, 94);

    private static Scalar lowergreen = new Scalar(0, 92, 148);
    private static Scalar uppergreen = new Scalar(255, 128, 214);

    private static Scalar lowergreenalt = new Scalar(0, 0, 0);
    private static Scalar uppergreenalt = new Scalar(255, 255, 255);

    private Mat primary = new Mat(); //YCrCb
    private Mat secondary = new Mat(); //LAB
    private Mat gray = new Mat();
    private Mat maskred = new Mat();
    private Mat maskblue = new Mat();
    private Mat maskgreen = new Mat();
    private Mat maskredalt = new Mat();
    private Mat maskbluealt = new Mat();
    private Mat maskgreenalt = new Mat();
    private Mat maskedGrayred = new Mat();
    private Mat maskedGrayblue = new Mat();
    private Mat maskedGraygreen = new Mat();
    private Mat redHierarchy = new Mat();
    private Mat blueHierarchy = new Mat();
    private Mat greenHierarchy = new Mat();

    public Color red = new Color(lowerred, upperred, lowerredalt, upperredalt);
    public Color blue = new Color(lowerblue, upperblue, lowerbluealt, upperbluealt);
    public Color green = new Color(lowergreen, uppergreen, lowergreenalt, uppergreenalt);

    @Override
    public Mat processFrame(Mat input) {
//        List<MatOfPoint> redContours = new ArrayList<MatOfPoint>();
//        List<MatOfPoint> blueContours = new ArrayList<MatOfPoint>();
//        List<MatOfPoint> greenContours = new ArrayList<MatOfPoint>();
//        Imgproc.cvtColor(input, primary, Imgproc.COLOR_RGB2YCrCb);
//        Imgproc.cvtColor(input, secondary, Imgproc.COLOR_RGB2Lab);
//        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
//
//        Core.inRange(primary, lowerred, upperred, maskred);
//        Core.inRange(primary, lowerblue, upperblue, maskblue);
//        Core.inRange(primary, lowergreen, uppergreen, maskgreen);
//        Core.inRange(secondary, lowerredalt, upperredalt, maskredalt);
//        Core.inRange(secondary, lowerbluealt, upperbluealt, maskbluealt);
//        Core.inRange(secondary, lowergreenalt, uppergreenalt, maskgreenalt);
//
//        Core.bitwise_and(maskred, maskredalt, maskedGrayred);
//        Core.bitwise_and(maskblue, maskbluealt, maskedGrayblue);
//        Core.bitwise_and(maskgreen, maskgreenalt, maskedGraygreen);
//
//        Imgproc.findContours(maskedGrayred, redContours,redHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.findContours(maskedGrayblue, blueContours,blueHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.findContours(maskedGrayblue, greenContours,blueHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

//
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
//            signalsRed = filterSignals(red, redContours, input.size().width, input.size().height);
//            signalsBlue = filterSignals(blue, blueContours, input.size().width, input.size().height);
//            signalsGreen = filterSignals(green, greenContours, input.size().width, input.size().height);
//        }
//        telemetry.addData("Red Signals",   signalsRed.size());
//        telemetry.addData("Blue Signals",  signalsBlue.size());
//        telemetry.addData("Green Signals", signalsGreen.size());

//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
//            conesRed = filterCones(red, redContours, input.size().width, input.size().height);
//        }
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
//            conesBlue = filterCones(blue, blueContours, input.size().width, input.size().height);
//        }
//        telemetry.addData("Red Cones",conesRed.size());
//        telemetry.addData("Blue Cones",conesBlue.size());
        
//        telemetry.update();
        return input;
    }

    @Override
    public void onViewportTapped() {
    }

}