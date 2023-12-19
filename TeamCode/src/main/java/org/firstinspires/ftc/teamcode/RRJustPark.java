package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "a")
//@Disabled

public class RRJustPark extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d purplePixelDropoff = new Pose2d(0,0,0);
        Pose2d straightenRobot = new Pose2d(0,0,0);
        Pose2d park = new Pose2d(0,0,0);
        Pose2d forwardPark = new Pose2d(0,0,0);
        Pose2d leftPark = new Pose2d(0,0,0);
        Pose2d initPose = new Pose2d(0,0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.RED);

        //extras.clawClose();
        //extras.wristMiddle();
        //sleep(500);
        //extras.initElevator();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        ExtraOpModeFunctions.Signal signal = ExtraOpModeFunctions.Signal.LEFT;

        while (!isStopRequested() && !opModeIsActive()) {
            signal = extras.telemetryTfod(ExtraOpModeFunctions.AutoStart.RR);
            telemetry.addData("Position", signal);
            telemetry.update();
            sleep(20);
        }

        switch(signal)
        {
            case LEFT:
                // move to LEFT column of parking tiles
                purplePixelDropoff = new Pose2d(22,17,Math.toRadians(23));
                straightenRobot = new Pose2d(14,0,Math.toRadians(0));
                break;

            case MIDDLE:
                // move to MIDDLE column of parking tiles
                purplePixelDropoff = new Pose2d(24,0,Math.toRadians(0));
                straightenRobot = new Pose2d(14,0,Math.toRadians(0));
                break;

            case RIGHT:
                // move to RIGHT colum of parking tiles
                purplePixelDropoff = new Pose2d(22,1,Math.toRadians(-20));
                straightenRobot = new Pose2d(14,0,Math.toRadians(0));
                break;

        }

        park = new Pose2d(0,-50,Math.toRadians(0));


        telemetry.addData("Place Pixel", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(purplePixelDropoff,Math.toRadians(0))
                        //.splineTo(purplePixelDropoff.position, purplePixelDropoff.heading)
                        //.strafeToLinearHeading(purplePixelDropoff.position, purplePixelDropoff.heading)
                        .build());

        sleep(2000);
        telemetry.addData("Back Up", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(straightenRobot.position, straightenRobot.heading)
                        //.splineToLinearHeading(straightenRobot,Math.toRadians(0))
                        .build());

        sleep(2000);
        telemetry.addData("All Done", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(park.position, park.heading)
                        .build());

        /*

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(5)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToY(-35)
                        .build());

         */



    }
    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


}