package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.math.Pose;

@TeleOp(name = "Swerve Test", group = "drive")
//@Disabled
public class LocalizationTest extends LinearOpMode {

    TwoWheelLocalizer swerveLocolizer;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        swerveLocolizer = new TwoWheelLocalizer(robot);

        swerveLocolizer.setPoseEstimate(new Pose2d(0,0,0));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.startIMUThread(this);

        robot.reset();

        waitForStart();

        while (!isStopRequested()) {

            swerveLocolizer.periodic();


            Pose poseEstimate = swerveLocolizer.getPos();
            telemetry.addData("X Coordinate :", poseEstimate.x);
            telemetry.addData("Y Coordinate :", poseEstimate.y);
            telemetry.addData("Heading :", poseEstimate.heading);
            telemetry.update();
        }
    }
}
