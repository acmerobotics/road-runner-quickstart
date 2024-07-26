package org.firstinspires.ftc.teamcode.Outdated_CenterStage.Our.RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

import java.lang.Math;
@Autonomous(name="Test", group="Linear OpMode")

@Disabled

public class ZTest extends LinearOpMode {


    int startPoseX = 72;
    int startPoseY = 0;
    int camera =3;
    private Servo claw;



    public Action close2() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(1);
                return false;
            }
        };

    }
    public Action open1() {
        Servo claw;
        claw= hardwareMap.get(Servo.class, "claw");
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(0);
                return false;
            }
        };

    }
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(startPoseX,startPoseY,Math.PI/2));


        waitForStart();

        while(opModeIsActive()) {
            if (camera == 3) {

                Actions.runBlocking(drive.actionBuilder(new Pose2d(startPoseX, startPoseY, Math.PI/2))
                                .lineToY(10)
                                .afterTime(.15,close2())
                                .lineToY(-10)
                                        .turnTo(Math.PI)
                        //try two after times
                        .build());
                requestOpModeStop();
            }



        }


    }
}



