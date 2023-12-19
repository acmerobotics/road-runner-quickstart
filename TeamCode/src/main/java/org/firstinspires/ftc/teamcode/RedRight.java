package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Collections;
import java.util.List;

@Config
@Autonomous(group = "a")
@Disabled

public class RedRight extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d purplePixelDropoff = new Pose2d(0,0,0);
        Pose2d initPose = new Pose2d(0,0,0);
        Pose2d moveToBoard = new Pose2d(0,0,0);
        Pose2d parkPosition = new Pose2d(0,0,0);

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
                purplePixelDropoff = new Pose2d(40,4,Math.toRadians(90));
                moveToBoard = new Pose2d(26,-28,Math.toRadians(90));
                parkPosition = new Pose2d(4,-28,Math.toRadians(90));
                break;

            case MIDDLE:
                // move to MIDDLE column of parking tiles
                purplePixelDropoff = new Pose2d(20,0,Math.toRadians(0));
                moveToBoard = new Pose2d(20,-46,Math.toRadians(90));
                parkPosition = new Pose2d(4,-46,Math.toRadians(90));
                break;

            case RIGHT:
                // move to RIGHT colum of parking tiles
                purplePixelDropoff = new Pose2d(20,-6,Math.toRadians(0));
                moveToBoard = new Pose2d(14,-46,Math.toRadians(90));
                parkPosition = new Pose2d(4,-46,Math.toRadians(90));
                break;

        }

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(purplePixelDropoff,Math.toRadians(0))
                        //.splineTo(purplePixelDropoff.position, purplePixelDropoff.heading)
                        //.strafeToLinearHeading(purplePixelDropoff.position, purplePixelDropoff.heading)
                        .build());

        //This is where we need to drop the purple pixel onto the spike board, please add code here :)

        safeWaitSeconds(2);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBoard.position, moveToBoard.heading)
                        .build());

        //add claw movement to drop the yellow pixel on the board

        safeWaitSeconds(2);

        //add robot movement to park into the correct position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPosition.position, parkPosition.heading)
                        .build());
    }
    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}