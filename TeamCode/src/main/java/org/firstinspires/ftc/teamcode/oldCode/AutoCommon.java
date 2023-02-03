package org.firstinspires.ftc.teamcode.oldCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AutoCommon extends LinearOpMode {

    protected RobotHardware robot;
    protected ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap, true);
        initialHeading = getHeading();
        telemetry.update();

        runtime.reset();
        telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cameraMagic();
        telemetry.update();

        waitForStart();
        telemetry.update();


    }
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


    public AprilTagDetection tagOfInterest = null;

    public void cameraMagic() {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
    private double inchesToTicks(double inches) {
        return inches * robot.DRIVE_MOTOR_TICKS_PER_ROTATION / (robot.WHEEL_DIAMETER * Math.PI);
    }

    private double initialHeading = 0;

    protected double getHeading() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - initialHeading;
    }

    protected double getHeadingDiff(double targetHeading) {
        double headingDiff = getHeading() - targetHeading;
        while (headingDiff > 180) {
            headingDiff -= 360;
        }
        while (headingDiff < -180) {
            headingDiff += 360;
        }
        return headingDiff;
    }


    protected void drive(double distance, double power) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) ;
        robot.driveTrain.stopMove();
    }

    protected void driveOnHeading(double distance, double power, double targetHeading) {
        targetHeading = -targetHeading;
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks && opModeIsActive()) {
            telemetry.addData("current heading", getHeading());
            double turnMod = getHeadingDiff(targetHeading) / 100;
            telemetry.addData("turn mod", turnMod);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
            telemetry.update();
        }
        robot.driveTrain.stopMove();
    }

    protected void driveOnHeadingRamp(double driveDistance, double minPower, double maxPower, double rampDistance, double targetHeading) {
        double dir = Math.signum(driveDistance);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(driveDistance));
        double rampTicks = inchesToTicks(Math.abs(rampDistance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(1, 0, 0, Math.abs(minPower) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            double startRampPower = minPower + (maxPower - minPower) * (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) / rampTicks);
            double endRampPower = minPower + (maxPower - minPower) * (Math.abs(encoderTicks - robot.driveTrain.motorFL.getCurrentPosition()) / (rampTicks * 2));
            double power = Range.clip(Math.min(startRampPower, endRampPower), minPower, maxPower);
            robot.driveTrain.startMove(Math.abs(power) * dir, 0, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }


    protected void strafeOnHeading(double distance, double power, double targetHeading) {
        double dir = Math.signum(distance * power);
        if (dir == 0) return;

        double encoderTicks = inchesToTicks(Math.abs(distance));

        robot.driveTrain.resetDriveEncoders();
        robot.driveTrain.startMove(0, 1, 0, Math.abs(power) * dir);
        while (Math.abs(robot.driveTrain.motorFL.getCurrentPosition()) < encoderTicks) {
            double turnMod = getHeadingDiff(targetHeading) / 100;
            robot.driveTrain.startMove(0, Math.abs(power) * dir, Range.clip(turnMod, -0.2, 0.2), 1);
        }
        robot.driveTrain.stopMove();
    }

    protected void turnToHeading(double targetHeading, double power) {
        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) { // added opMode=true &&
            robot.driveTrain.startMove(0, 0, 1, power * Math.signum(getHeadingDiff(targetHeading)));
        }
        robot.driveTrain.stopMove();
    }

    protected void hover() {
        robot.lift.motorLiftL.setTargetPosition(300);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(300);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift. motorLiftR.setPower(1);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        robot.lift.currentState = RobotHardware.Lift.States.HOVER;
    }

    protected void highJunction() {
        robot.lift.motorLiftL.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
        robot.lift.currentState = RobotHardware.Lift.States.HighJunction;
//                elapsedTime.reset();
    }
    protected void highJunctionDrop() {
        robot.lift.motorLiftL.setTargetPosition(2400);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(0.5);
        robot.lift.motorLiftR.setTargetPosition(2400);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(0.5);
    }
    protected void turretLeft135() {

        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_LEFT_135_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
        robot.lift.servoExtension.setPosition(0.65);
        robot.lift.currentState = RobotHardware.Lift.States.TurretLeft;
    }
    protected void turretLeft45() {
        robot.lift.motorLiftL.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_LEFT_45_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
    }

    protected void turretRight45() {
        robot.lift.motorLiftL.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(robot.lift.LIFT_HIGH_POS);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_RIGHT_45_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
    }

    protected void turretStackPos() {
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_STACK_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
        robot.lift.currentState = RobotHardware.Lift.States.TurretLeft;
    }

    protected void turretRight135() {
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_RIGHT_135_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_AUTO_POS);
        robot.lift.currentState = RobotHardware.Lift.States.TurretLeft;
    }
    protected void turret0Pos() {
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_0_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.7);
        robot.lift.servoExtension.setPosition(robot.lift.TURRET_0_POS);
        robot.lift.currentState = RobotHardware.Lift.States.TurretLeft;
    }

    protected void intake() {
        robot.lift.motorLiftL.setTargetPosition(0);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(0.7);
        robot.lift.motorLiftR.setTargetPosition(0);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(0.7);
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_0_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.5);
        robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
        robot.lift.currentState = RobotHardware.Lift.States.INTAKE;
    }

    protected void liftPos(int pos) {
            robot.lift.motorLiftL.setTargetPosition(pos);
            robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.motorLiftL.setPower(0.7);
            robot.lift.motorLiftR.setTargetPosition(pos);
            robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.motorLiftR.setPower(0.7);
            robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_0_POS);
            robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.motorTurret.setPower(0.5);
//            robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
            robot.lift.currentState = RobotHardware.Lift.States.INTAKE;
        }

    protected void turretLeft90() {
        robot.lift.motorLiftL.setTargetPosition(robot.lift.LIFT_MID_POS);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(robot.lift.LIFT_MID_POS);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
        robot.lift.motorTurret.setTargetPosition(robot.lift.TURRET_LEFT_POS);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.7);
    }

    protected void turretLeftLow90() {
        robot.lift.motorLiftL.setTargetPosition(robot.lift.LIFT_LOW_POS);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(1);
        robot.lift.motorLiftR.setTargetPosition(robot.lift.LIFT_LOW_POS);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(1);
        robot.lift.motorTurret.setTargetPosition(1690);
        robot.lift.motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorTurret.setPower(0.7);
    }

    protected void midJunctionDrop() {
        robot.lift.motorLiftL.setTargetPosition(1200);
        robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftL.setPower(0.5);
        robot.lift.motorLiftR.setTargetPosition(1200);
        robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.motorLiftR.setPower(0.5);
    }
    }




//    protected void turnToHeading(double targetHeading, double power) {
//        while (opModeIsActive() && Math.abs(getHeadingDiff(targetHeading)) > 6) {
//            robot.driveTrain.startMove(0, 0, 1, power * Math.signum(getHeadingDiff(targetHeading)));
//        }
//        robot.driveTrain.stopMove();
//    }
//
//    public void armUp(int towerPos) {
//        if (opModeIsActive()) {
//            if (towerPos == 1) {
//                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_TOP);
//                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.arm.motorArm.setPower(0.2);
//                robot.arm.currentState = RobotHardware.Arm.States.UP;
//                robot.arm.elapsedTime.reset();
//            } else if (towerPos == 2) {
//                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_MID_HUB_POSITION);
//                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.arm.motorArm.setPower(0.2);
//                robot.arm.currentState = RobotHardware.Arm.States.UP;
//                robot.arm.elapsedTime.reset();
//            } else if (towerPos == 3) {
//                robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_LOW_HUB_POSITION);
//                robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.arm.motorArm.setPower(0.2);
//                robot.arm.currentState = RobotHardware.Arm.States.UP;
//                robot.arm.elapsedTime.reset();
//            }
//        }
//    }
//    public void armDown() {
//        robot.arm.servoFlicker.setPosition(robot.arm.FLICKER_CLOSED);
//        robot.arm.servoArm.setPosition(robot.arm.SERVO_HOVER);
//        robot.arm.motorArm.setTargetPosition(robot.arm.MOTOR_BOTTOM);
//        robot.arm.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.arm.motorArm.setPower(0.05);
//        robot.arm.currentState = RobotHardware.Arm.States.INTAKE;
//        robot.arm.elapsedTime.reset();
//    }




