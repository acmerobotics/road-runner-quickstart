/*
         * Copyright (c) 2021 OpenFTC Team
         *
         * Permission is hereby granted, free of charge, to any person obtaining a copy
         * of this software and associated documentation files (the "Software"), to deal
         * in the Software without restriction, including without limitation the rights
         * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
         * copies of the Software, and to permit persons to whom the Software is
         * furnished to do so, subject to the following conditions:
         *
         * The above copyright notice and this permission notice shall be included in all
         * copies or substantial portions of the Software.
         * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
         * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
         * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
         * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
         * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
         * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
         * SOFTWARE.
         */

        package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.oldCode.AutoCommon;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class ParkingONLY extends AutoCommon
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // int ID_TAG_OF_INTEREST = 18;Tag ID 18 from the 36h11 family

    //int ID tag 1,2,3 from 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;


//    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        super.runOpMode();

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//        }
//
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//
////        /* Actually do something useful */
////        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
////            /*
////             * Insert your autonomous code here, presumably running some default configuration
////             * since the tag was never sighted during INIT
////             */
////        }
////        if(tagOfInterest == null) {
////            /*
////             * Insert your autonomous code here, presumably running some default configuration
////             * since the tag was never sighted during INIT
////             */
////        }
////        else
////        {
////            /*
////             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
////             */
////
////            // e.g.
////            if(tagOfInterest.pose.x <= 20)
////            {
////                // do something
////            }
////            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
////            {
////                // do something else
////            }
////            else if(tagOfInterest.pose.x >= 50)
////            {
////                // do something else
////            }
////        }

        if (tagOfInterest == null) {
            //trajetory (code that makes it move)
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(1000);
            hover();
            sleep(1000);
            driveOnHeading(50,0.3,0);
            sleep(500);
            driveOnHeading(-10,0.3,0);
            sleep(500);
            intake();
            sleep(1000);
        } else if (tagOfInterest.id == LEFT) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(1000);
            hover();
            sleep(1000);
            driveOnHeading(50,0.3,0);
            sleep(500);
            driveOnHeading(-10,0.3,0);
            sleep(500);
            turnToHeading(90,0.3);
            driveOnHeading(34,0.3,-90);
            turnToHeading(0,0.3);
            driveOnHeading(10,0.3,0);
            intake();
            sleep(1000);
        } else if (tagOfInterest.id == MIDDLE) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(1000);
            hover();
            sleep(1000);
            driveOnHeading(50,0.3,0);
            sleep(500);
            driveOnHeading(-10,0.3,0);
            sleep(500);
            intake();
            sleep(1000);
        } else if (tagOfInterest.id == RIGHT) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(1000);
            hover();
            sleep(1000);
            driveOnHeading(50,0.3,0);
            sleep(500);
            driveOnHeading(-10,0.3,0);
            sleep(500);
            turnToHeading(90,0.3);
            driveOnHeading(-33,0.3,-90);
            turnToHeading(0,0.3);
            driveOnHeading(10,0.3,0);
            intake();
            sleep(1000);
            //trajetory (code that makes it move)
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
     //   while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
