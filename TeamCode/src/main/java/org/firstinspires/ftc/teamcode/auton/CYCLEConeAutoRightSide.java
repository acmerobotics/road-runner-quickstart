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
public class CYCLEConeAutoRightSide extends AutoCommon
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

        if (tagOfInterest == null) {
            //trajetory (code that makes it move)
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            hover();
            sleep(250);
            driveOnHeading(72,0.3,0);
            sleep(500);
            highJunction();
            sleep(250);
            turretLeft135();
            sleep(2500);
            highJunctionDrop();
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            // DROPS PRELOAD

            sleep(250);
            highJunction();
            driveOnHeading(-2.5,0.3,0);
            intake();
            sleep(1000);
            turnToHeading(90,0.3);
            sleep(500);
            liftPos(570);
            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
            driveOnHeading(-15,0.3,-90);
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            liftPos(1200);
            //PICKS UP FIRST STACK CONE

            driveOnHeading(11,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

            //DROPS FIRST STACK CONE

            highJunction();
            sleep(1000);
            intake();
            sleep(2000);
            liftPos(450);
            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
            driveOnHeading(-15.5,0.3,-90);
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(500);
            liftPos(1200);
            //PICKUP SECOND CONE

            driveOnHeading(21,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            highJunction();
            sleep(1000);
            intake();
            driveOnHeading(5.5,0.5,-90);



            sleep(10000000);
        } else if (tagOfInterest.id == LEFT) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            hover();
            sleep(250);
            driveOnHeading(72,0.3,0);
            sleep(500);
            highJunction();
            sleep(250);
            turretLeft135();
            sleep(2500);
            highJunctionDrop();
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            // DROPS PRELOAD

            sleep(250);
            highJunction();
            driveOnHeading(-2.5,0.3,0);
            intake();
            sleep(1000);
            turnToHeading(90,0.3);
            sleep(500);
            liftPos(570);
            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
            driveOnHeading(-15,0.3,-90);
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            liftPos(1200);
            //PICKS UP FIRST STACK CONE

            driveOnHeading(11,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

            //DROPS FIRST STACK CONE

            highJunction();
            sleep(1000);
            intake();
            sleep(2000);
            liftPos(450);
            robot.lift.servoExtension.setPosition(0.74);
            driveOnHeading(-15.5,0.3,-90);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(750);
            liftPos(1200);
            //PICKUP SECOND CONE

            driveOnHeading(21,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            highJunction();
            sleep(1000);
            intake();
            driveOnHeading(31,0.6,-90);



            sleep(10000000);
        } else if (tagOfInterest.id == MIDDLE) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            hover();
            sleep(250);
            driveOnHeading(72,0.3,0);
            sleep(500);
            highJunction();
            sleep(250);
            turretLeft135();
            sleep(2500);
            highJunctionDrop();
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            // DROPS PRELOAD

            sleep(250);
            highJunction();
            driveOnHeading(-2.5,0.3,0);
            intake();
            sleep(1000);
            turnToHeading(90,0.3);
            sleep(500);
            liftPos(570);
            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
            driveOnHeading(-15,0.3,-90);
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            liftPos(1200);
            //PICKS UP FIRST STACK CONE

            driveOnHeading(11,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

            //DROPS FIRST STACK CONE

            highJunction();
            sleep(1000);
            intake();
            sleep(2000);
            liftPos(450);
            robot.lift.servoExtension.setPosition(0.74);
            driveOnHeading(-15.5,0.3,-90);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(750);
            liftPos(1200);
            //PICKUP SECOND CONE

            driveOnHeading(21,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            highJunction();
            sleep(1000);
            intake();
            driveOnHeading(5.5,0.8,-90);



            sleep(10000000);
        } else if (tagOfInterest.id == RIGHT) {
            driveOnHeading(2,0.2,0);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            hover();
            sleep(250);
            driveOnHeading(72,0.3,0);
            sleep(500);
            highJunction();
            sleep(250);
            turretLeft135();
            sleep(2500);
            highJunctionDrop();
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            // DROPS PRELOAD

            sleep(250);
            highJunction();
            driveOnHeading(-2.5,0.3,0);
            intake();
            sleep(1000);
            turnToHeading(90,0.3);
            sleep(500);
            liftPos(570);
            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
            driveOnHeading(-15,0.3,-90);
            sleep(250);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(250);
            liftPos(1200);
            //PICKS UP FIRST STACK CONE

            driveOnHeading(11,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);

            //DROPS FIRST STACK CONE

            highJunction();
            sleep(1000);
            intake();
            sleep(2000);
            liftPos(450);
            robot.lift.servoExtension.setPosition(0.74);
            driveOnHeading(-15.5,0.3,-90);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            sleep(750);
            liftPos(1200);
            //PICKUP SECOND CONE

            driveOnHeading(21,0.3,-90);
            highJunction();
            turretRight135();
            sleep(2500);
            highJunctionDrop();
            sleep(500);
            robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
            highJunction();
            sleep(1000);
            intake();
            driveOnHeading(-12,1,-90);


            sleep(10000000);
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
