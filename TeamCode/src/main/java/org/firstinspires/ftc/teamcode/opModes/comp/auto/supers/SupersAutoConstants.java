package org.firstinspires.ftc.teamcode.opModes.comp.auto.supers;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class SupersAutoConstants {
    //in inches
//THIS IS FOR RED HIGH BASKET!!!!!!!!!!!!!!!!!
//scoring the pre-load one

    public final double ROBOT_WIDTH = 18;
    public final double ROBOT_LENGTH = 16;
    public final Pose2d RED_STARTING_POSITION = new Pose2d(-12, -62,0);
    public final Pose2d RED_PRELOAD_DROP = new Pose2d(-10, -33, 0);
    public final Pose2d RED_POST_DROP = new Pose2d(-10, -42, Math.toRadians(270));
    public final double RED_POST_DROP_TANGENT = Math.toRadians(180);


    //picking up and scoring the 1st one (1st meaning the first one that we picked up I guess this is actually the 2nd one)
    public final Pose2d RED_PICKUP_ONE = new Pose2d(-48, -48, Math.toRadians(270));
    public final double RED_PICKUP_ONE_TANGENT = Math.toRadians(70);
    public final Pose2d RED_HIGH_BASKET = new Pose2d(-56, -56, Math.toRadians(225));
    public final double RED_HIGH_BASKET_TANGENT = Math.toRadians(-180);



    //picking up and scoring the 2nd one
    public final Pose2d RED_PICKUP_TWO = new Pose2d(-58, -48, Math.toRadians(270));
    public final double RED_PICKUP_TWO_TANGENT = Math.toRadians(-180);


    //picking up and scoring the 3rd one
    public final Pose2d RED_PICKUP_THREE = new Pose2d(-56, -44, Math.toRadians(310));
    public final double RED_PICKUP_THREE_TANGENT = Math.toRadians(-180);


















    //THIS IS FOR BLUE HIGH BASKET!!!!!!!!!!!!!!!!!

    //VERY MUCH UNDER CONSTRUCTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//scoring the pre-load one
    public static final Pose2d BLUE_STARTING_POSITION = new Pose2d(12, 62, 0);
    public static final Vector2d BLUE_PRELOAD_DROP = new Vector2d(10, 33);
    public static final Pose2d BLUE_POST_DROP = new Pose2d(10, 42, Math.toRadians(90));
    public static final double BLUE_POST_DROP_TANGENT = Math.toRadians(180);


    //picking up and scoring the 1st one (1st meaning the first one that we picked up I guess this is actually the 2nd one)
    public static final Pose2d BLUE_PICKUP_ONE = new Pose2d(48, 48, Math.toRadians(90));
    public static final double BLUE_PICKUP_ONE_TANGENT = Math.toRadians(70);
    public static final Pose2d BLUE_HIGH_BASKET = new Pose2d(56, 56, Math.toRadians(45));
    public static final double BLUE_HIGH_BASKET_TANGENT = Math.toRadians(90);



    //picking up and scoring the 2nd one
    public static final Pose2d BLUE_PICKUP_TWO = new Pose2d(58, 48, Math.toRadians(90));
    public static final double BLUE_PICKUP_TWO_TANGENT = Math.toRadians(90);


    //picking up and scoring the 3rd one
    public static final Pose2d BLUE_PICKUP_THREE = new Pose2d(56, 44, Math.toRadians(125));
    public static final double PICKUP_THREE_TANGENT = Math.toRadians(90);



}
