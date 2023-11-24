package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.CenterStage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Config
public class FindTheServoPosistions extends LinearOpMode {

    private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private Pose2d startPose = new Pose2d(3,4,Math.toRadians(80));

    private Pose2d endPose = new Pose2d(15,4,Math.toRadians(70));



    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStarted()) {
            telemetry.addData("Servo Posistion ", drive.leftGripServo.getPosition());
            telemetry.addData("claw Posistion" , drive.rightGripServo.getPosition());
            telemetry.update();
        }

    }



}
