package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.math.Pose;


@Config
public class Globals {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static int LEFTSIDE_REGION_X = 25;
    public static int LEFTSIDE_REGION_Y = 170;

    public static int RIGHTSIDE_REGION_X = 200;
    public static int RIGHTSIDE_REGION_Y = 165;

    public static int REGION_WIDTH = 75;
    public static int REGION_HEIGHT = 55;

    public static Pose yummypose = new Pose(0, 0, 0);
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    // Lift Subsystem Constants
    public static int LIFT_HIGH_POS = 597;
    public static int LIFT_MID_POS = 365;
    public static int LIFT_RETRACT_POS = -5;

    // in/s and in/s^2
    public static double LIFT_MAX_V = 100;
    public static double LIFT_MAX_A = 99999;
    public static double LIFT_MAX_D = 250;

    public static double LIFT_LATCHED = 0.78;
    public static double LIFT_INTERMEDIATE = 0.795;
    public static double LIFT_UNLATCHED = 0.885;

    public static final double LIFT_TICKS_PER_INCH = 22.754;

    public static double LIFT_MANUAL_FACTOR = 1;
    public static double LIFT_MIN = 0;
    public static double LIFT_MAX = 0;

    public static double LIFT_EXTENDED_TOLERANCE = 20;
    public static double LIFT_ERROR_TOLERANCE = 20;

    // Intake Subsystem Constants
    public static double INTAKE_EXTENSION_MAX_V = 50;
    public static double INTAKE_EXTENSION_MAX_A = 50;
    public static double INTAKE_EXTENSION_MAX_D = 50;

    public static double INTAKE_FOURBAR_MAX_V = 7;
    public static double INTAKE_FOURBAR_MAX_A = 9;
    public static double INTAKE_FOURBAR_MAX_D_UP = 7;
    public static double INTAKE_FOURBAR_MAX_D_DOWN = 7;

    public static double INTAKE_CLAW_OPEN = 0.39;
    public static double INTAKE_CLAW_CLOSED = 0.31;
    public static double INTAKE_CLAW_OPEN_AUTO = 0.43;
    public static double INTAKE_CLAW_CLEAR = 0.5;

    public static double INTAKE_PIVOT_LOW = 0.355;
    public static double INTAKE_PIVOT_FLAT = 0.5125;
    public static double INTAKE_PIVOT_FLAT_AUTO = 0.5125;
    public static double INTAKE_PIVOT_PRE_TRANSFER = 0.63;
    public static double INTAKE_PIVOT_TRANSFER = 0.62;
    public static double INTAKE_PIVOT_PICKUP = 0.5;

    public static double INTAKE_TURRET_OUTWARDS = 0.728;
    public static double INTAKE_TURRET_INTERMEDIATE = 0.449;
    public static double INTAKE_TURRET_INWARDS = 0.17;

    public static double INTAKE_FOURBAR_PRE_TRANSFER = 0.40;
    public static double INTAKE_FOURBAR_TRANSFER = 0.472;
    public static double INTAKE_FOURBAR_INTERMEDIATE = 0.33;
    public static double INTAKE_FOURBAR_CLEAR_POS = 0.38;
    public static double INTAKE_FOURBAR_INTAKE = 0.035;
    public static double INTAKE_FOURBAR_LOW = 0.359;
    public static double INTAKE_FOURBAR_GROUND = 0.06;
    public static double INTAKE_FOURBAR_FALLEN = 0.085;

    public static double F_OFFSET = 0.0;

    public static long INTAKE_CLAW_CLOSE_TIME = 150; // ms

    public static final double EXTENSION_TICKS_PER_INCH = 23.54;

    public static double INTAKE_MANUAL_FACTOR = -250;
    public static double INTAKE_TURRET_FACTOR = 0.007;
    public static double INTAKE_FOURBAR_FACTOR = -0.005;
    public static double INTAKE_PIVOT_FACTOR = 0.0025;
    public static double INTAKE_MIN = 0;
    public static double INTAKE_MAX = 577;

    public static double INTAKE_EXTENDED_TOLERANCE = 5;
    public static double INTAKE_ERROR_TOLERANCE = 20;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
    public static boolean MANUAL_ENABLED = true;
    public static boolean HAS_AUTO_ERROR = false;
    public static boolean IS_PARKING = false;
    public static boolean USE_WHEEL_FEEDFORWARD = false;

    public static boolean SWERVE_X = false;

    public static int wait1 = 100;
    public static int wait2 = 300;
    public static int wait3 = 50;
    public static int wait4 = 50;
    public static int wait5 = 200;
    public static int wait6 = 500;
}
