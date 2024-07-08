package org.firstinspires.ftc.teamcode.overload;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Drake was here :)

import org.firstinspires.ftc.teamcode.MecanumDrive;


//import org.firstinspires.ftc.teamcode.code.util.BackupHardware;


@TeleOp(name="DriverCentric", group = "Drive")
public class driverCentric extends LinearOpMode {

    //Class def
    double deflator;
    double magnitude;
    double theta;
    double x;
    double y;
    double angVel;
    double thetaOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        waitForStart();


        while (opModeIsActive()) {
            mecDrive.updatePoseEstimate();

            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;
            magnitude = Math.sqrt(Math.pow(-gamepad1.left_stick_x,2) + Math.pow(-gamepad1.left_stick_y,2));

            theta = thetaOffset + (-gamepad1.left_stick_y <= 0 && gamepad1.left_stick_x < 0 ? Math.atan(gamepad1.left_stick_x/-gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() - Math.PI:-gamepad1.left_stick_y <= 0 ? Math.atan(gamepad1.left_stick_x/-gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() + Math.PI: Math.atan(gamepad1.left_stick_x/-gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble());

            x = (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : Math.cos(theta)) *magnitude*deflator;
            y = gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : -Math.sin(theta)*magnitude*deflator;
            angVel = -gamepad1.right_stick_x * deflator;
            mecDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                x,
                                y
                        ),
                    angVel
                    ));

        }
    }
}
