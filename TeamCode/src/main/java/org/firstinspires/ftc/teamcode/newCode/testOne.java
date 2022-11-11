

package org.firstinspires.ftc.teamcode.newCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.newCode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.ArrayList;

@Autonomous
public class testOne extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    //17348 Code
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor leftRear;
    private DcMotor ViperMotor;
    private Servo gripper;

    int LeftPos;
    int RightPos;
    int LBPos;
    int RBPos;
    int LinearPos;
    int leftTarget;
    int rightTarget;
    int LBack;
    int RBack;
    double speed;



    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //17348 Code
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        ViperMotor = hardwareMap.get(DcMotor.class, "ViperMotor");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setPosition(0.4);
        sleep(1000);
        ViperMotor.setPower(-0.5);
        sleep(40);
        ViperMotor.setPower(0);
        RightPos = 0;
        LeftPos = 0;
        LBPos = 0;
        RBPos = 0;
        LinearPos = 0;

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
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

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }




        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive(1050, -1050, -1050, 1050, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(2200, 2200, 2200, 2200, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(-580, 580, 580, -580, 0.25);
            sleep(1000);
            reset_values();
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ViperMotor.setPower(-0.5);
            sleep(4300);
            ViperMotor.setPower(0);
            drive(300, 300, 300, 300, 0.25);
            sleep(2000);
            gripper.setPosition(0.6);
            reset_values();
            sleep(2000);
            drive(-200, -200, -200, -200, 0.25);
            gripper.setPosition(0.4);
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ViperMotor.setPower(0.5);
            sleep(1500);
            reset_values();
            sleep(1000);
            ViperMotor.setPower(0);
            drive(-1800, 1800, 1800, -1800, 0.25);

        }else if(tagOfInterest.id == MIDDLE){
            drive(1050, -1050, -1050, 1050, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(2200, 2200, 2200, 2200, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(-580, 580, 580, -580, 0.25);
            sleep(1000);
            reset_values();
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ViperMotor.setPower(-0.5);
            sleep(4300);
            ViperMotor.setPower(0);
            drive(300, 300, 300, 300, 0.25);
            sleep(2000);
            gripper.setPosition(0.6);
            reset_values();
            sleep(2000);
            drive(-200, -200, -200, -200, 0.25);
            gripper.setPosition(0.4);
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ViperMotor.setPower(0.5);
            sleep(1500);
            reset_values();
            sleep(1000);
            ViperMotor.setPower(0);
            drive(-700, 700, 700, -700, 0.25);
        }
        else
        {
            drive(1050, -1050, -1050, 1050, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(2200, 2200, 2200, 2200, 0.25);
            sleep(1000);
            reset_values();
            sleep(1000);
            drive(-580, 580, 580, -580, 0.25);
            sleep(1000);
            reset_values();
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ViperMotor.setPower(-0.5);
            sleep(4300);
            ViperMotor.setPower(0);
            drive(300, 300, 300, 300, 0.25);
            sleep(2000);
            gripper.setPosition(0.6);
            reset_values();
            sleep(2000);
            drive(-200, -200, -200, -200, 0.25);
            gripper.setPosition(0.4);
            ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ViperMotor.setPower(0.5);
            sleep(1500);
            reset_values();
            sleep(1000);
            ViperMotor.setPower(0);
            drive(700, -700, -700, 700, 0.25);

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}
    }

    private void reset_values()
    {
        int leftTarget;
        int rightTarget;
        int LBack;
        int RBack;
        int Linear_Target;

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightPos = 0;
        LeftPos = 0;
        LBPos = 0;
        RBPos = 0;
        leftTarget = 0;
        rightTarget = 0;
        RBack = 0;
        LBack = 0;
        LinearPos = 0;
        Linear_Target = 0;
    }

    private void drive(int leftTarget, int rightTarget, int LBack, int RBack, double speed)
    {
        LeftPos += leftTarget;
        RightPos += rightTarget;
        LBPos += LBack;
        RBPos += RBack;
        rightFront.setTargetPosition(rightTarget);
        leftFront.setTargetPosition(leftTarget);
        rightRear.setTargetPosition(RBack);
        leftRear.setTargetPosition(LBack);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(speed);
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);
        while (opModeIsActive() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy())
        {
            idle();
        }
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

