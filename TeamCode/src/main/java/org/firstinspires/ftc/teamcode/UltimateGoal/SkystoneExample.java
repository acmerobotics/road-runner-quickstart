package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SkystoneExample extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorSimple liftR;
    private DcMotorSimple liftL;
    private ServoImplEx SSGrabber;
    private CRServo grabber;
    private DistanceSensor wallSensorRed;
    DigitalChannel bottomSwitch;
    double SS;
    SampleMecanumDrive drive;

    MecanumConstraints constraints = new MecanumConstraints(
            new DriveConstraints(70.0, 40.0, 0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0),
            18.9, 14.0);


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);


        liftR = hardwareMap.get(DcMotorSimple.class, "liftR");
        liftL = hardwareMap.get(DcMotorSimple.class, "liftL");
        grabber = hardwareMap.get(CRServo.class, "grabber");
        wallSensorRed = hardwareMap.get(DistanceSensor.class,"wallSensorRed");
        bottomSwitch = hardwareMap.get(DigitalChannel.class, "bottomSwitch");
//        SSGrabber =  hardwareMap.get(ServoImplEx.class, "SSGrabber");

        // LP = negative moves lift up
        // LP = positive moves lift down

//        SSGrabber.setPwmRange(new PwmControl.PwmRange(750, 2250));



        waitForStart();



//        splineTest();
//        goToFoundationLine();
//        liftPower(-.5, .5);
//        PIDTurn(90);
//        sleep(250);
//        PIDforward(20);
//        dropBlock();
//        liftPower(.6, .25);
//        PIDstrafeLeft(6);
//        PIDback(48);
//        liftPower(-.6, .25);
//        PIDforward(2);
//        PIDback(5);
//        PIDstrafeRight(40);
//        PIDforward(24);
//        PIDTurn(88);
//        getBlock();
//        PIDforward(10);




    }


    public void PIDforward(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory);
        }
    }

    public void PIDTurn(double angle) {

        if (opModeIsActive()) {
            drive.turn(Math.toRadians(angle));
        }
    }

    public void PIDstrafeLeft(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectoryAsync(trajectory);
        }
    }

    public void PIDstrafeRight(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectoryAsync(trajectory);
        }
    }

    public void PIDback(double Distance) {

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .back(Distance)
                .build();

        if (opModeIsActive()) {

            if (isStopRequested()) return;

            drive.followTrajectoryAsync(trajectory);
        }
    }

    public void getBlock() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);


    }

    public void liftPower(double lP, double time) {
        double startTime = runtime.seconds();
        if (opModeIsActive()) {

            if (bottomSwitch.getState() == true) {
                telemetry.addData("limit switch", "Is not Pressed");
            } else if (bottomSwitch.getState() == false) {
                lP = Range.clip(lP, -1, 0); //motors are reversed
                telemetry.addData("limit switch", "Is pressed");
            }
            liftL.setPower(lP);
            liftR.setPower(-lP);
            while (startTime + time >= runtime.seconds() && opModeIsActive()) {
                //do nothing
            }
            liftL.setPower(0);
            liftR.setPower(0);
        }
    }

    public void grabBlock() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 2 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);
    }

    public void releaseBlock() {
        if (opModeIsActive()) {
            dropBlock();
        }
    }

    public void dropBlock() {
        if (opModeIsActive()) {
            grabber.setPower(.75);
            double startTime = runtime.seconds();
            while (startTime + 1 >= runtime.seconds() && opModeIsActive()) {
                //do nothing
            }
            grabber.setPower(0);
        }
    }

    public void runTo2ndSkystone() {
        if (opModeIsActive()) {
            if (SS == -1) {
                //middle
                PIDforward(73);
                PIDTurn(-88);
                getBlock();

            } else if (SS == 0) {
                //right
                PIDforward(66);
                PIDTurn(-88);
                getBlock();

            } else if (SS == 1) {
                //left
                PIDforward(73);
                PIDTurn(-88);
                getBlock();


            }
        }

    }

    public void getSkystoneServo() {
        if (opModeIsActive()) {
            SSGrabber.setPosition(1);
        }

    }

    public void dropSkystoneServo() {
        if (opModeIsActive()) {
            SSGrabber.setPosition(.5);
        }
    }

    public void grabFoundation() {
        grabber.setPower(-.75);
        double startTime = runtime.seconds();
        while (startTime + 3 >= runtime.seconds() && opModeIsActive()) {
            //do nothing
        }
        grabber.setPower(0);
    }

    public void strafeDistanceSensor() {
        while (wallSensorRed.getDistance(DistanceUnit.INCH) < 20.5 && opModeIsActive()) {
            drive.setMotorPowers(.2, -.2, .2, -.2);
        }

    }
    public void foundation() {
        if (opModeIsActive()) {
            sleep(100);
            double x = drive.getPoseEstimate().getX();
            double y = drive.getPoseEstimate().getY();
            drive.followTrajectoryAsync(
                    drive.trajectoryBuilder(new Pose2d())
                            .lineTo(new Vector2d(x, -83))
                            .build()
            );
            sleep(200);
            PIDTurn(90);
            sleep(200);
            double startTimeDrive = runtime.seconds();
            drive.setMotorPowers(.2, .2, .2, .2);
            while (startTimeDrive + .8 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.4);
                grabber.setPower(-.5);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            dropBlock();
            PIDforward(4);
            liftPower(.7, .5);
            PIDback(54);
            strafeDistanceSensorFoundation();

        }
    }
    public void liftMovement(double lP) {
        if (opModeIsActive()) {

            if (bottomSwitch.getState() == true) {
                telemetry.addData("limit switch", "Is not Pressed");
            } else if (bottomSwitch.getState() == false) {
                lP = Range.clip(lP, -1, 0); //motors are reversed
                telemetry.addData("limit switch", "Is pressed");
            }
            liftL.setPower(lP);
            liftR.setPower(-lP);

        }
    }
    public void strafeDistanceSensorFoundation() {
        if (wallSensorRed.getDistance(DistanceUnit.INCH) <= 26) {
            double startTime = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTime + .6 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.5);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }

            double startTimedrive = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTimedrive + .5 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(.65);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
        }
        else {
            double startTimedrive = runtime.seconds();
            while (wallSensorRed.getDistance(DistanceUnit.INCH) <= 38 && startTimedrive + .5 >= runtime.seconds() && opModeIsActive()) {
                liftMovement(-.65);
                drive.setMotorPowers(.4, -.4, .4, -.4);
            }
            drive.setMotorPowers(0, 0, 0, 0);
            liftMovement(0);
            liftPower(.7,.5);

        }
    }

}