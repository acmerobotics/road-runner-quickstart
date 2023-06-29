package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.math.Pose;

//@Disabled
@TeleOp(name = "Localization Test (odo) ", group = "drive")
public class LocalizationTest extends LinearOpMode {

    TwoWheelLocalizer swerveLocolizer;

//    private BrainSTEMRobot robot = BrainSTEMRobot.getInstance();

    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        BrainSTEMRobot robot = new BrainSTEMRobot();

        swerveLocolizer = new TwoWheelLocalizer(robot);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        robot.reset();

        robot.init(hardwareMap, telemetry);

        waitForStart();

//        while (opModeInInit()) {
//            swerveLocolizer.setPoseEstimate(new Pose2d(0,0,0));
//            robot.startIMUThread(this);
//            PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            PhotonCore.experimental.setMaximumParallelCommands(8);
//            PhotonCore.enable();
//        }

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
