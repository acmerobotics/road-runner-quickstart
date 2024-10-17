package org.firstinspires.ftc.teamcode.Testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.time.Clock;
import java.util.Timer;

@Autonomous
public class ActionTesting extends LinearOpMode {

    public class A {
        public MecanumDrive drive;

        ElapsedTime time = new ElapsedTime();

        public A() {
            drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        }
        public class Spin360 implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    time.reset();
                    drive.rightFront.setPower(0.5);
                    init = true;
                }

                if (time.seconds() < 2) {
                    return true;
                }
                drive.rightFront.setPower(0);
                return false;
            }
        }
        public Action spin360() {
            return new Spin360();
        }

        public class Forward5Sec implements Action {
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!init) {
                    time.reset();
                    drive.rightFront.setPower(0.5);
                    drive.rightBack.setPower(0.5);
                    drive.leftFront.setPower(0.5);
                    drive.leftBack.setPower(0.5);
                }

                if (time.seconds() < 1) {
                    return true;
                }
                drive.rightFront.setPower(0);
                drive.rightBack.setPower(0);
                drive.leftFront.setPower(0);
                drive.leftBack.setPower(0);
                return false;
            }
        }
        public Action forward5Sec() {
            return new Forward5Sec();
        }




    }

    @Override
    public void runOpMode() {
        A a = new A();

        TrajectoryActionBuilder c = a.drive.actionBuilder(a.drive.pose)
                        .lineToX(20);

        TrajectoryActionBuilder d = a.drive.actionBuilder(a.drive.pose)
                        .strafeTo(new Vector2d(40, 5));


        Action cc = c.build();
        Action dd = d.build();
        waitForStart();

        Actions.runBlocking(new SequentialAction(
                cc,
                a.spin360(),
                dd,
                a.forward5Sec()

        ));
    }
}
