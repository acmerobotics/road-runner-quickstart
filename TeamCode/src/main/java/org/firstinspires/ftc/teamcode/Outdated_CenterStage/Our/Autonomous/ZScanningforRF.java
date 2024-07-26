package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Autonomous(name="ScanningforRF", group="Linear OpMode")

@Disabled
public class ZScanningforRF extends LinearOpMode {

    //Motor and Servo Assignment
    private DcMotor BACK_R;
    private DcMotor FRONT_L;
    private DcMotor FRONT_R;
    private DcMotor BACK_L;
    private Servo claw;
    //private Servo claw2;
    private DcMotor shoulder;

    //Sensors
    private IMU imu_IMU;
    private DistanceSensor distance;

    //Speed Variable
    float Speed_Movement_Multiplier;

    //Wheel variables influenced by IMU
    AngularVelocity Angular_Velocity;
    YawPitchRollAngles Orientation2;
    double Gyro_Degrees;
    double Gyro_Radians;

    // Shoulder Variables
    double shoulderStick;
    int shoulder_Position;
    int PixelLevel = 1;
    int ShoulderLevel;

    //Encoder Movement
    double DistanceInCM;
    double circumference = Math.PI * 9.6;
    double rotations = DistanceInCM/circumference;
    int encoderTarget = (int) (rotations *537.6);
    double WheelSpeed;
    int StepNum = 1;

    //IMU influenced Turn/Yaw
    int Angle;

    //Distance Sensor
    double DistanceSensor;

    //Camera
    private AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    double HorizontalOffset;

    private void InitialSetup () {

        Speed_Movement_Multiplier = 0.4f;
        Angle = 0;

        //Wheel Setup
        BACK_L = hardwareMap.get(DcMotor.class, "BACK_L");
        BACK_R = hardwareMap.get(DcMotor.class, "BACK_R");
        FRONT_L = hardwareMap.get(DcMotor.class, "FRONT_L");
        FRONT_R = hardwareMap.get(DcMotor.class, "FRONT_R");
        shoulder = hardwareMap. get(DcMotor.class, "shoulder");
        claw = hardwareMap. get(Servo.class, "claw");
        //claw2 = hardwareMap.get(Servo.class, "claw2");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        claw.scaleRange(0, 1);
        //claw2.scaleRange(0,1);
        //claw2.setPosition(1);
        claw.setPosition(1);

        FRONT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRONT_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRONT_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BACK_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IMU();
        Camera();

    }
    private void IMU () {

        //Imu Initialize and Reset
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu_IMU.resetYaw();
    }

    private void Camera () {

        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        //builder.setCamera(BuiltinCameraDirection.BACK);
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(1280, 720));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private void MoveForward () {

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void ReleasePurplePixel () {
        claw.setPosition(0);
    }
    /*

    private void ReleaseYellowPixel () {
        claw2.setPosition(0);
    }

    */
    private void MoveRight () {

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(-encoderTarget);
        BACK_R.setTargetPosition(-encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void MoveLeft () {

        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void MoveBackward () {

        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(-encoderTarget);
        BACK_R.setTargetPosition(-encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void TurnRight () {

        FRONT_R.setPower(-WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(-WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        if (Gyro_Degrees >= Angle) {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
        }

    }
    private void TurnLeft () {

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(-WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(-WheelSpeed);

        if (Gyro_Degrees <= Angle) {
            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);
        }

    }
    private void RaiseArm () {

        shoulder.setTargetPosition(-300);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);
        //where timer would start and eventually cause claw to release pixel


    }
    private void LowerArm () {

        shoulder.setTargetPosition(0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(1);

    }
    private void IdandDistanceCorrectPlacePixel () {
        RaiseArm();
        WheelSpeed = .5;
        DistanceInCM = 3;
        MoveForward();
        //ReleaseYellowPixel();
        DistanceInCM = 5;
        MoveBackward();
        LowerArm();
        Angle = 0;
        TurnRight();
    }
    private void MoveForwardUntilDistanceReachedLeftSpike () {

        if(DistanceSensor>6 && DistanceSensor < 8) {
            ScanforLeftTag();
        }

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void MoveForwardUntilDistanceReachedMiddleSpike () {

        if(DistanceSensor>6 && DistanceSensor < 8) {
            ScanforMiddleTag();
        }

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
    private void MoveForwardUntilDistanceReachedRightSpike () {

        if(DistanceSensor>6 && DistanceSensor < 8) {
            ScanForRightTag();
        }

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
    private void ScanforLeftTag () {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            if (detection.id ==1) {
                HorizontalOffset = detection.ftcPose.x;
                DistanceInCM = 2.54*HorizontalOffset;
                //NOT really "Rightward" direction, this is if it is in a weird spot
                MoveRight();
                DistanceInCM = 5;
                MoveLeft();
                IdandDistanceCorrectPlacePixel();
            }

        }

        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void ScanforMiddleTag () {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            if (detection.id ==2) {
                HorizontalOffset = detection.ftcPose.x;
                DistanceInCM = 2.54*HorizontalOffset;
                //NOT really "Rightward" direction, this is if it is in a weird spot
                MoveRight();
                IdandDistanceCorrectPlacePixel();
            }

        }
        FRONT_R.setTargetPosition(-encoderTarget);
        FRONT_L.setTargetPosition(encoderTarget);
        BACK_R.setTargetPosition(encoderTarget);
        BACK_L.setTargetPosition(-encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void ScanForRightTag () {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            if (detection.id ==3) {
                HorizontalOffset = detection.ftcPose.x;
                DistanceInCM = 2.54*HorizontalOffset;
                MoveRight();
                DistanceInCM = 5;
                MoveRight();
                IdandDistanceCorrectPlacePixel();
            }

        }

        FRONT_R.setTargetPosition(encoderTarget);
        FRONT_L.setTargetPosition(-encoderTarget);
        BACK_R.setTargetPosition(-encoderTarget);
        BACK_L.setTargetPosition(encoderTarget);

        FRONT_R.setPower(WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        FRONT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRONT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BACK_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void TurnRightandMove () {

        FRONT_R.setPower(-WheelSpeed);
        FRONT_L.setPower(WheelSpeed);
        BACK_R.setPower(-WheelSpeed);
        BACK_L.setPower(WheelSpeed);

        //if left spike
        if(Gyro_Degrees<-30 && DistanceSensor < 75) {

            //Reset gyro to 0 degrees
            FRONT_R.setPower(-WheelSpeed);
            FRONT_L.setPower(WheelSpeed);
            BACK_R.setPower(-WheelSpeed);
            BACK_L.setPower(WheelSpeed);

            if (Gyro_Degrees < 2 && Gyro_Degrees > -2) {
                FRONT_R.setPower(0);
                FRONT_L.setPower(0);
                BACK_R.setPower(0);
                BACK_L.setPower(0);
            }

            WheelSpeed = .85;
            DistanceInCM = 45;
            MoveForward();
            Angle = 90;
            TurnRight();
            DistanceInCM = 8;
            MoveForward();
            ReleasePurplePixel();
            DistanceInCM = 8;
            MoveBackward();
            DistanceInCM =25;
            MoveLeft();
            Angle = 90;
            TurnRight();
            DistanceInCM = 200;
            WheelSpeed =.9;
            MoveForwardUntilDistanceReachedLeftSpike();


            //if middle spike
        } else if (Gyro_Degrees>-20 && Gyro_Degrees <20 && DistanceSensor < 75 ) {

            //Reset gyro to 0 degrees
            FRONT_R.setPower(-WheelSpeed);
            FRONT_L.setPower(WheelSpeed);
            BACK_R.setPower(-WheelSpeed);
            BACK_L.setPower(WheelSpeed);

            if (Gyro_Degrees > -2 && Gyro_Degrees < 2) {
                FRONT_R.setPower(0);
                FRONT_L.setPower(0);
                BACK_R.setPower(0);
                BACK_L.setPower(0);
            }

            DistanceInCM = 10;
            MoveRight();
            DistanceInCM = 43;
            MoveForward();
            ReleasePurplePixel();
            DistanceInCM = 8;
            MoveBackward();
            Angle = 90;
            TurnRight();
            WheelSpeed = .9;
            DistanceInCM = 190;
            MoveForward();
            DistanceInCM = 10;
            MoveLeft();
            DistanceInCM = 200;
            WheelSpeed =.9;
            MoveForwardUntilDistanceReachedMiddleSpike();

            //if right spike
        } else if (Gyro_Degrees>30 && DistanceSensor < 75 ) {

            //Reset gyro to 0 degrees
            FRONT_R.setPower(WheelSpeed);
            FRONT_L.setPower(-WheelSpeed);
            BACK_R.setPower(WheelSpeed);
            BACK_L.setPower(-WheelSpeed);

            if (Gyro_Degrees < 2 && Gyro_Degrees > -2) {
                FRONT_R.setPower(0);
                FRONT_L.setPower(0);
                BACK_R.setPower(0);
                BACK_L.setPower(0);
            }

            //Move Right
            DistanceInCM = 45;
            WheelSpeed = .9;
            MoveForward();
            Angle = 90;
            TurnRight();
            DistanceInCM = 8;
            MoveForward();
            ReleasePurplePixel();
            DistanceInCM = 8;
            MoveBackward();
            DistanceInCM = 25;
            MoveRight();
            WheelSpeed = .9;
            DistanceInCM = 210;
            MoveForward();
            DistanceInCM = 8;
            MoveRight();
            DistanceInCM = 200;
            WheelSpeed =.9;
            MoveForwardUntilDistanceReachedRightSpike();


        } else if (Gyro_Degrees >= Angle) {

            FRONT_R.setPower(0);
            FRONT_L.setPower(0);
            BACK_R.setPower(0);
            BACK_L.setPower(0);

        }

    }

    private void Telemetry () {

        telemetry.addData("Gyro Degrees", Gyro_Degrees);
        telemetry.addData("Gyro Radians", Gyro_Radians);
        telemetry.addData("Yaw", Orientation2.getYaw(AngleUnit.DEGREES));
        telemetry.addData("FR", FRONT_R.getPower());
        telemetry.addData("FL", FRONT_L.getPower());
        telemetry.addData("BR", BACK_R.getPower());
        telemetry.addData("BL", BACK_L.getPower());
        telemetry.addData("shoulder", shoulder.getCurrentPosition());
        telemetry.addData("shoulderStick", shoulderStick);
        telemetry.addData("shoulder_Position", shoulder_Position);
        telemetry.addData("Pixel Level: ", PixelLevel);
        telemetry.addData("Shoulder Level:", ShoulderLevel);
        telemetry.addData("Distance", DistanceInCM);
        //telemetry.addData("shoulder", shoulder.getTargetPosition());
        telemetry.update();

    }
    @Override
    public void runOpMode() {

        InitialSetup();

        // Set Directions
        BACK_L.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_R.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_L.setDirection(DcMotorSimple.Direction.REVERSE);
        FRONT_R.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset shoulder Encoder
        shoulder_Position = 0;

        IMU();

        waitForStart();

        while (opModeIsActive()) {

            //Distance Sensor
            DistanceSensor = distance.getDistance(DistanceUnit.CM);

            //shoulderEncoder
            shoulder_Position = shoulder.getCurrentPosition();

            //Imu Angle Variables
            Angular_Velocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
            Orientation2 = imu_IMU.getRobotYawPitchRollAngles();
            Gyro_Degrees = Orientation2.getYaw(AngleUnit.DEGREES);

            // 1. Move Forward
            StepNum = 1;
            DistanceInCM = 31.5;
            WheelSpeed = 1;
            MoveForward();
            telemetry.addData("Step: ",StepNum);

            // 2. Rotate to Left
            StepNum = 2;
            WheelSpeed = .8;
            Angle = -90;
            TurnLeft();

            telemetry.addData("Step: ",StepNum);

            // 3. Check for Team Prop Rotate While rotating to the right and Finish Autonomous
            StepNum = 3;
            WheelSpeed = .3;
            Angle = 90;
            TurnRightandMove();
            telemetry.addData("Step: ",StepNum);

            // Telemetry
            Telemetry();

        }

    }


}
