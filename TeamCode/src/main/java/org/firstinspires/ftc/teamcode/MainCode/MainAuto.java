package org.firstinspires.ftc.teamcode.MainCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.MainCode.Constants.Spike;
import org.firstinspires.ftc.teamcode.MainCode.Constants.Alliance;
import org.firstinspires.ftc.teamcode.MainCode.Constants.Side;
import org.firstinspires.ftc.teamcode.MainCode.Constants.Park;

@Config
@TeleOp(name="Autonomous", group="Linear Opmode")

public final class MainAuto extends LinearOpMode {
    public static Side start;
    public static Spike lcr;
    public static Alliance color;
    public static Park park;

    //for dashboard
    public static String startValue = "";
    public static String lcrValue = "";
    public static String colorValue = "";
    public static String parkValue = "";


    public void runOpMode() throws InterruptedException {
        Pose2d startingPose;
        Pose2d nextPose;
        double xOffset = 0;
        double yOffset = 0;
        MecanumDrive drive;
        double reflect;
        ConfigDashboard();
//TODO on blue side, flip left and right as they are mirrored and wrong values
        waitForStart();
        if (color.equals(Alliance.RED)) {
            reflect = 1.0;
        } else {
            reflect = -1.0;
        }
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class) && start.equals(Side.BACKSTAGE)) { //BackstageSide
                startingPose = new Pose2d(12, -64*reflect, Math.toRadians(90*reflect));
                drive = new MecanumDrive(hardwareMap, startingPose);
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(2 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(12 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
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
                switch (lcr) {
                    case LEFT:
                        nextPose = new Pose2d(-46 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case CENTER:
                        nextPose = new Pose2d(-36 + xOffset, -26*reflect + yOffset, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;
                    case RIGHT:
                        nextPose = new Pose2d(-26 + xOffset, -30*reflect + yOffset, Math.toRadians(90*reflect));
                        Actions.runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .splineToConstantHeading(nextPose.position, nextPose.heading)
                                        .build());
                        break;

                }
            }
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