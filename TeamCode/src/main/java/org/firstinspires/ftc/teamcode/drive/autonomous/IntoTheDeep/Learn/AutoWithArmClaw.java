package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.IntoTheDeep.Learn;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BlueAuto", group="practice")
public class AutoWithArmClaw extends LinearOpMode {
    private boolean pathFinished = false;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35, -60, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        Slide slide = new Slide(hardwareMap, telemetry);
        Pivot pivot = new Pivot(hardwareMap, telemetry);

        Action move1 = drive.actionBuilder(new Pose2d(60, -35, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 34))
                .waitSeconds(0.1)
                .build();

        Action move2 = drive.actionBuilder(new Pose2d(60, -35, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-56, 50), Math.toRadians(90))
                .waitSeconds(.1)
                .build();

        Action move3 = drive.actionBuilder(new Pose2d(-56, 50, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(0, 34), Math.toRadians(270))
                .build();

        Action move4 = drive.actionBuilder(new Pose2d(0, 34, Math.toRadians(90)))
                .strafeTo(new Vector2d(-60, 65))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            if (!pathFinished) {
                Actions.runBlocking(
                        move1
                );
                pathFinished = true;
            }
            else {
                return;
            }
        }
    }

    public class HangSpecimen implements Action {
        private final Slide slide;
        private final Claw claw;
        private final Pivot pivot;
        private final int slidePosition;
        private final double slidePower = 0.5;

        public HangSpecimen(Slide slide, Claw claw, Pivot pivot, int slidePosition) {
            this.slide = slide;
            this.claw = claw;
            this.pivot = pivot;
            this.slidePosition = slidePosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pivot.pivotToPosition(-300, 0.75, 5);
            slide.moveToPosition(700, 0.75);
            return false;
        }
    }
    public Action HangSpecimen(Slide slide, Claw claw, Pivot pivot, int slidePosition) {
        return new HangSpecimen(slide, claw, pivot, slidePosition);
    }
}