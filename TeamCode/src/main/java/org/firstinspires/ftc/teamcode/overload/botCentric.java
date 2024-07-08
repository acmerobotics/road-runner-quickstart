package org.firstinspires.ftc.teamcode.overload;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;


//import org.firstinspires.ftc.teamcode.code.util.BackupHardware;


@TeleOp(name="BotCentric", group = "Drive")
public class botCentric extends LinearOpMode {

    //Class def
    double deflator;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        MecanumDrive mecDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        while (opModeIsActive()) {

            deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;


            mecDrive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y*deflator,
                                -gamepad1.left_stick_x*deflator
                        ),
                    -gamepad1.right_stick_x * deflator
                    ));

        }
    }
}
