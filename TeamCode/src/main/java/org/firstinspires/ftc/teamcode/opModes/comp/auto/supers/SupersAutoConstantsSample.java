package org.firstinspires.ftc.teamcode.opModes.comp.auto.supers;
import com.acmerobotics.roadrunner.Pose2d;


public class SupersAutoConstantsSample {


    public final double ROBOT_WIDTH = 18;
    public final double ROBOT_LENGTH = 16;
    public static final Pose2d STARTING_POSITION = new Pose2d(12, 61.5, Math.toRadians(270));
    public static final Pose2d PRELOAD_DROP = new Pose2d(10, 34.5, Math.toRadians(270));

    //PUSH 1
    public static final Pose2d BLOCK_ONE_A = new Pose2d(10, 42, Math.toRadians(90));
    public static final Pose2d BLOCK_ONE_B = new Pose2d(48, 40, Math.toRadians(90));
    public static final Pose2d BLOCK_ONE_C = new Pose2d(53, 53, Math.toRadians(270));
    public static final double BLOCK_ONE_A_TANGENT = Math.toRadians(360);
    public static final double BLOCK_ONE_B_TANGENT = Math.toRadians(-70);
    public static final double BLOCK_ONE_C_TANGENT = Math.toRadians(90);

    //PUSH 2
    public static final Pose2d BLOCK_TWO_A = new Pose2d(58, 40, Math.toRadians(90));
    public static final Pose2d BLOCK_TWO_B = new Pose2d(53, 53, Math.toRadians(45));
    public static final double BLOCK_TWO_A_B_TANGENT = Math.toRadians(90);

    //PUSH 3
    public static final Pose2d BLOCK_THREE_A = new Pose2d(56, 25, Math.toRadians(180));
    public static final Pose2d BLOCK_THREE_B = new Pose2d(53, 53, Math.toRadians(45));
    public static final double BLOCK_THREE_A_TANGENT = Math.toRadians(-90);
    public static final double BLOCK_THREE_B_TANGENT = Math.toRadians(90);


}