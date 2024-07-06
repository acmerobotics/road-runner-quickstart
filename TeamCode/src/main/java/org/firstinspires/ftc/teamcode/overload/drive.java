package org.firstinspires.ftc.teamcode.overload;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Drake was here :)
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;


//import org.firstinspires.ftc.teamcode.code.util.BackupHardware;


@TeleOp(name="Drive", group = "TeleOp")
public class drive extends LinearOpMode {

    //Class def




    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()){}

        while (opModeIsActive()) {
            MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
            double deflator;
            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;


            mecDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y*deflator,
                                -gamepad1.left_stick_x*deflator
                        ),
                        -gamepad1.right_stick_x*deflator
                    ));

        }
    }
}
