

package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.testCode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.testCode.driveTest;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class autonomousVisionTest extends LinearOpMode {
    //Drive Train Constants


        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotorEx leftFront = null;
        private DcMotorEx leftRear = null;
        private DcMotorEx rightFront = null;
        private DcMotorEx rightRear = null;
        private Servo intake = null;
        private DcMotor arm = null;
        private List<DcMotorEx> motors;

        static final double FORWARD_SPEED = 0.6;
        static final double TURN_SPEED = 0.5;

        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            for (DcMotorEx motor : motors) {
                motor.setZeroPowerBehavior(zeroPowerBehavior);
            }
        }


        //vision constants
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;
        driveTest driveTest;


        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!

        double fx = 822.317; //578.272;
        double fy = 822.317; //578.272;
        double cx = 319.495; //402.145;
        double cy = 242.502; //221.506;

        // UNITS ARE METERS
        double tagsize = 0.05;//0.166;

        //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

        // Tag ID 1,2,3 from the 36h11 family
        int LEFT = 0;
        int MIDDLE = 1;
        int RIGHT = 2;
        AprilTagDetection tagOfInterest = null;

        @Override
        public void runOpMode() {
            //Drive Train Init
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRear.setDirection(DcMotorSimple.Direction.FORWARD);


            motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

            setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            telemetry.setMsTransmissionInterval(50);

            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested()) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() != 0) {
                    boolean tagFound = false;

                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");

                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }

            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */




            //May be add here
            //waitForStart();





            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }


            if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                //input trajectory
                leftFront.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                rightRear.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }


                leftFront.setPower(-TURN_SPEED);
                leftRear.setPower(TURN_SPEED);
                leftRear.setPower(-TURN_SPEED);
                rightFront.setPower(TURN_SPEED);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3)) {
                    telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }



                // Step 4:  Stop
                leftFront.setPower(0);
                leftRear.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);

                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(1000);



            } else if (tagOfInterest.id == MIDDLE) {
                leftFront.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                rightRear.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                // Step 4:  Stop
                leftFront.setPower(0);
                leftRear.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);

                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(1000);
            } else {
                leftFront.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                rightRear.setPower(FORWARD_SPEED);
                leftRear.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }


                leftFront.setPower(TURN_SPEED);
                leftRear.setPower(-TURN_SPEED);
                leftRear.setPower(TURN_SPEED);
                rightFront.setPower(-TURN_SPEED);

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3)) {
                    telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                    telemetry.update();
                }



                // Step 4:  Stop
                leftFront.setPower(0);
                leftRear.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);

                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(1000);
            }


            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
            // while (opModeIsActive()) {sleep(20);}
        }

        void tagToTelemetry(AprilTagDetection detection) {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }


}
