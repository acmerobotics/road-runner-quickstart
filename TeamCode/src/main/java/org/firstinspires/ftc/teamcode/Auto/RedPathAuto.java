package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedPathAuto extends LinearOpMode {

            @Override
            public void runOpMode() {
                Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
                MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

                // vision here that outputs position
                int visionOutputPosition = 1;

                TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(10, 10));



                while (!isStopRequested() && !opModeIsActive()) {
                    int position = visionOutputPosition;
                    telemetry.addData("Position during Init", position);
                    telemetry.update();
                }

                int startPosition = visionOutputPosition;
                telemetry.addData("Starting Position", startPosition);
                telemetry.update();
                waitForStart();

                if (isStopRequested()) return;


            }
}



