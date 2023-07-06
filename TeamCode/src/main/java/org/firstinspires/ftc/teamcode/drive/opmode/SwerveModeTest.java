package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.robot.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Point;
import org.firstinspires.ftc.teamcode.util.math.Pose;

//@Disabled
@TeleOp(name = "Swerve Mode Test", group = "drive")
public class SwerveModeTest extends LinearOpMode {

    TwoWheelLocalizer swerveLocolizer;

    private BrainSTEMRobot robot = new BrainSTEMRobot();


    private ElapsedTime timer;

//    private SwerveDrivetrain swerveDrive;

    private SlewRateLimiter forward;

    private SlewRateLimiter strafe;

    private boolean lock_robot_heading = false;

    private double targetHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

//        BrainSTEMRobot robot = new BrainSTEMRobot();

        SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(robot);


        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        robot.init(hardwareMap, telemetry);

        waitForStart();

        if (isStopRequested()) return;


        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        waitForStart();

        while (opModeInInit()) {

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            Constants.AUTO = false;
//            Constants.USING_IMU = true;
//            Constants.USE_WHEEL_FEEDFORWARD = false;

            robot.init(hardwareMap, telemetry);

//            robot.enabled = true;

//            swerveLocolizer.setPoseEstimate(new Pose2d(0,0,0));
//            robot.startIMUThread(this);

//            forward = new SlewRateLimiter(5);
//            strafe = new SlewRateLimiter(5);


//            PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//            PhotonCore.experimental.setMaximumParallelCommands(8);
//            PhotonCore.enable();
        }

        while (!isStopRequested()) {

//            robot.read(swerveDrivetrain);

            Pose drive = new Pose(0,0,0);

            swerveDrivetrain.set(drive);




//            if (gamepad1.right_bumper) {
//                swerveDrivetrain.setReset(true);
//            } else if (gamepad1.left_bumper) {
//                swerveDrivetrain.setReset(false);
//            }

//            swerveLocolizer.periodic();
//            robot.loop(drive, swerveDrivetrain);
            swerveDrivetrain.write();
            swerveDrivetrain.updateModules();
//            robot.write(swerveDrivetrain);


//            Pose poseEstimate = swerveLocolizer.getPos();
//            telemetry.addData("X Estimate :", poseEstimate.x);
//            telemetry.addData("Y Estimate :", poseEstimate.y);
//            telemetry.addData("Heading Estimate :", poseEstimate.heading);
            telemetry.addLine("--------------------");
            telemetry.addData("swerve Forward :",swerveDrivetrain.isForward());
//            telemetry.addData("swerve reset :", swerveDrivetrain.isReset());
            telemetry.update();

            robot.clearBulkCache();
        }
    }

}
