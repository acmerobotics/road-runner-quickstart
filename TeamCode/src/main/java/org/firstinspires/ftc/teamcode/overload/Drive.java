package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;



@TeleOp(name="Drive", group = "Drive")
public class Drive extends LinearOpMode {

    //Class def


    double deflator;
    double magnitude;
    double theta;
    double x;
    double y;
    double angVel;
    double thetaOffset = 0;

    boolean driveCentric;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        waitForStart();


        while (opModeIsActive()) {
            mecDrive.updatePoseEstimate();

            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;

            if (gamepad1.b) {
                driveCentric = false;
            } else if (gamepad1.a) {
                driveCentric = true;
            }

            if (driveCentric) {

                magnitude = Math.sqrt(Math.pow(-gamepad1.left_stick_x, 2) + Math.pow(-gamepad1.left_stick_y, 2));

                theta = thetaOffset + (-gamepad1.left_stick_y <= 0 && gamepad1.left_stick_x < 0 ? Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() - Math.PI : -gamepad1.left_stick_y <= 0 ? Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble() + Math.PI : Math.atan(gamepad1.left_stick_x / -gamepad1.left_stick_y) + mecDrive.pose.heading.toDouble());

                x = (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : Math.cos(theta)) * magnitude * deflator;
                y = gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 ? 0 : -Math.sin(theta) * magnitude * deflator;

                telemetry.addLine("MODE: Driver Centric");
            } else {
                x = -gamepad1.left_stick_y;
                y = -gamepad1.left_stick_x;

                telemetry.addLine("MODE: Bot Centric");
            }

            angVel = -gamepad1.right_stick_x * deflator;

            telemetry.update();

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
