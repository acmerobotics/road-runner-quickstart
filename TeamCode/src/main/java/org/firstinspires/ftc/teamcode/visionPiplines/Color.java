package org.firstinspires.ftc.teamcode.visionPiplines;
import org.opencv.core.*;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import java.util.List;
public class Color {
    Scalar lower;
    Scalar upper;
    Scalar loweralt;
    Scalar upperalt;
    public Color(Scalar lower, Scalar upper, Scalar loweralt, Scalar upperalt) {
        this.lower = lower;
        this.upper = upper;
        this.loweralt = loweralt;
        this.upperalt = upperalt;
    }
}