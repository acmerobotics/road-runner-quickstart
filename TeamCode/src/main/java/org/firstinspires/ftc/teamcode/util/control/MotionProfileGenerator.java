package org.firstinspires.ftc.teamcode.util.control;

public class MotionProfileGenerator {
    public static MotionProfile generateSimpleMotionProfile(double start, double end, double maxvel, double maxaccel) {
        return new MotionProfile(start, end, maxvel, maxaccel);
    }
}
