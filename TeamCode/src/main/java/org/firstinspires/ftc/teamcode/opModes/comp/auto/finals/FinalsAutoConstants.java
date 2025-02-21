package org.firstinspires.ftc.teamcode.opModes.comp.auto.finals;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public class FinalsAutoConstants {
    //in inches
//THIS IS FOR RED SAMPLE SIDE!!!!!!!!!!!!!!!!!
//scoring the pre-load one


    public final double ROBOT_WIDTH = 18;
    public final double ROBOT_LENGTH = 16;
    public static final Pose2d STARTING_POSITION = new Pose2d(-12, 61.5, Math.toRadians(270));
    public static final Pose2d PRELOAD_DROP = new Pose2d(-4, 32.25, Math.toRadians(270));

    //PUSH 1
//    .setTangent(Math.toRadians(180))
    public static final Pose2d PUSH_ONE_A = new Pose2d(-28, 39, Math.toRadians(260));
    public static final Pose2d PUSH_ONE_B = new Pose2d(-37, 34, Math.toRadians(260));
    public static final Pose2d PUSH_ONE_C = new Pose2d(-37, 48, Math.toRadians(170));
    public static final double PUSH_ONE_A_B_TANGENT = Math.toRadians(180);
    public static final double PUSH_ONE_C_TANGENT = Math.toRadians(90);

    //PUSH 2
    public static final Pose2d PUSH_TWO_A = new Pose2d(-44, 33, Math.toRadians(240));
    public static final Pose2d PUSH_TWO_B = new Pose2d(-44, 49, Math.toRadians(165));
    public static final double PUSH_TWO_A_TANGENT = Math.toRadians(180);
    public static final double PUSH_TWO_B_TANGENT = Math.toRadians(90);

    //PUSH 3
    public static final Pose2d PUSH_THREE_A = new Pose2d(-51, 33, Math.toRadians(230));
    public static final Pose2d PUSH_THREE_B = new Pose2d(-50, 47, Math.toRadians(165));
    public static final Pose2d PUSH_THREE_C = new Pose2d(-43, 63, Math.toRadians(270));
    public static final double PUSH_THREE_A_TANGENT = Math.toRadians(180);
    public static final double PUSH_THREE_B_C_TANGENT = Math.toRadians(90);


    //HANG 1
    public static final Pose2d HANG_ONE_A = new Pose2d(-6, 33, Math.toRadians(270));
    public static final double HANG_ONE_A_TANGENT = Math.toRadians(270);
    public static final Pose2d HANG_ONE_B = new Pose2d(-45, 63.5, Math.toRadians(270));
    public static final double HANG_ONE_B_TANGENT = Math.toRadians(90);
    public static final double HANG_ONE_A_SET_TANGENT = Math.toRadians(270);
    public static final double HANG_ONE_B_SET_TANGENT = Math.toRadians(90);

    //HANG 2
    public static final Pose2d HANG_TWO_A = new Pose2d(-8, 33, Math.toRadians(270));
    public static final double HANG_TWO_A_TANGENT = Math.toRadians(270);
    public static final Pose2d HANG_TWO_B = new Pose2d(-45, 63.5, Math.toRadians(270));
    public static final double HANG_TWO_B_TANGENT = Math.toRadians(90);
    public static final double HANG_TWO_A_SET_TANGENT = Math.toRadians(90);

    //HANG 3
    public static final Pose2d HANG_THREE_A = new Pose2d(-10, 33, Math.toRadians(270));
    public static final double HANG_THREE_A_TANGENT = Math.toRadians(270);
    public static final Pose2d HANG_THREE_B = new Pose2d(1, 31.25, Math.toRadians(270));
    public static final double HANG_THREE_B_TANGENT = Math.toRadians(90);
    public static final double HANG_THREE_A_SET_TANGENT = Math.toRadians(90);

    //HANG 4
    public static final Pose2d HANG_FOUR_A = new Pose2d(-12, 33, Math.toRadians(270));
    public static final double HANG_FOUR_A_TANGENT = Math.toRadians(270);


}