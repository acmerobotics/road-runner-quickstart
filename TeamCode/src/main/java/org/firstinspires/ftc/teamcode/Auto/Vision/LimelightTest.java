package org.firstinspires.ftc.teamcode.Auto.Vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class LimelightTest extends LinearOpMode {

    @Override
    public void runOpMode() {





       // MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose);

      //  Pose2d StartPose = new Pose2d(x,y, Math.toRadians(0));


        Limelight limelight = new Limelight(hardwareMap);
        waitForStart();

        while (opModeIsActive()){

            }


        }





    }



