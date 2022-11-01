package org.firstinspires.ftc.teamcode.visionPiplines;
import org.opencv.core.*;
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