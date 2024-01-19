package org.firstinspires.ftc.teamcode.MainCode.Autonomous;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision.VisionHandler;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Vision.VisionParameters;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Spike;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Alliance;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Side;
import org.firstinspires.ftc.teamcode.MainCode.Autonomous.Constants.Park;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
@TeleOp(name="Autonomous", group="Linear Opmode")


public final class MainAuto extends LinearOpMode {
    public static Side start = Side.AUDIENCE;
    public static Spike lcr;
    public static Alliance color = Alliance.RED;
    public static Park park = Park.CORNER;

    //for dashboard
    public static String startValue = "";
    public static String lcrValue = "";
    public static String colorValue = "";
    public static String parkValue = "";
    VisionHandler visionHandler;


    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        double xOffset = 0;
        double yOffset = 0;
        MecanumDrive drive;
        int reflect;
        int LCRNUM = 0;
        visionHandler = new VisionHandler();
        ConfigDashboard();


        while(!isStarted()){
            if (gamepad1.right_bumper){
                if(color.equals(Alliance.RED)){
                    color = Alliance.BLUE;
                } else {
                    color = Alliance.RED;
                }
            }
            if (gamepad1.left_bumper){
                if(park.equals(Park.CORNER)){
                    park = Park.STAGE;
                } else {
                    park = Park.CORNER;
                }
            }
            if (gamepad1.a){
                if(start.equals(Side.AUDIENCE)){
                    start = Side.BACKSTAGE;
                } else {
                    start = Side.AUDIENCE;
                }
            }
            telemetry.addData("Color: ", color.name());
            telemetry.addData("Side: ", start.name());
            telemetry.addData("Parking: ", park.name());
            telemetry.update();
        }
        waitForStart();

        visionHandler.init(hardwareMap);
        waitForStart();

        if(color.equals(Alliance.RED)){
            visionHandler.setRed();
        }else{
            visionHandler.setBlue();
        }
        visionHandler.setLeft();
        double left = visionHandler.read();
        visionHandler.setMiddle();
        double mid = visionHandler.read();
        visionHandler.setRight();
        double right = visionHandler.read();
        if(left >= mid && left >= right)
            lcr = Spike.LEFT;
        if(mid >= right && mid >= left)
            lcr = Spike.CENTER;
        if(right >= left && right >= mid)
            lcr = Spike.RIGHT;

        if (color.equals(Alliance.RED)) {
            reflect = 1;
        } else {
            reflect = -1;
        }
        switch (lcr){
            case LEFT:
                LCRNUM = -1*reflect;
                break;
            case CENTER:
                LCRNUM = 0;
                break;
            case RIGHT:
                LCRNUM = 1*reflect;
                break;
        }
        if (start.equals(Side.BACKSTAGE)){ //BackstageSide
            startingPose = new Pose2d(12, -64*reflect, Math.toRadians(90*reflect));
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (LCRNUM) {
                case -1:
                    nextPose = new Pose2d(2 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 0:
                    nextPose = new Pose2d(12 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 1:
                    nextPose = new Pose2d(22 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
            }
        } else { //AudienceSide
            startingPose = new Pose2d(-36, -64, Math.PI / 2);
            drive = new MecanumDrive(hardwareMap, startingPose);
            switch (LCRNUM) {
                case -1:
                    nextPose = new Pose2d(-46 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 0:
                    nextPose = new Pose2d(-36 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;
                case 1:
                    nextPose = new Pose2d(-26 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineToConstantHeading(nextPose.position, nextPose.heading)
                                    .build());
                    break;

            }
        }
        //go to backboard
        if (start.equals(Side.BACKSTAGE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(90*reflect))
                            .lineToY(-48*reflect)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .turnTo(0)
                            .splineToConstantHeading(new Vector2d(45, -36*reflect), 0)
                            .build());
        } else if (start.equals(Side.AUDIENCE)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turnTo(Math.toRadians(90*reflect))
                            .lineToY(-60*reflect)
                            .build());
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setTangent(0)
                            .turnTo(0)
                            .lineToX(24)
                            .splineToConstantHeading(new Vector2d(45, -36*reflect), 0)
                            .build());
        }
    }



    private static void ConfigDashboard() {
        switch (startValue){
            case "AUDIENCE":
                start = Side.AUDIENCE;
                break;
            case "BACKSTAGE":
                start = Side.BACKSTAGE;
                break;
        }
        switch (colorValue) {
            case "RED":
                color = Alliance.RED;
                break;
            case "BLUE":
                color = Alliance.BLUE;
                break;
        }
        switch (lcrValue) {
            case "LEFT":
                lcr = Spike.LEFT;
                break;
            case "CENTER":
                lcr = Spike.CENTER;
                break;
            case "RIGHT":
                lcr = Spike.RIGHT;
                break;
        }
        switch (parkValue) {
            case "CORNER":
                park = Park.CORNER;
                break;
            case "STAGE":
                park = Park.STAGE;
        }
    }
}
