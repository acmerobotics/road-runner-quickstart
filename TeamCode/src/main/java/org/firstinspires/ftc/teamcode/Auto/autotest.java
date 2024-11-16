package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RED_LEFT", group = "Autonomous")
public class autotest extends LinearOpMode {
//    public class Lift {
//        private DcMotorEx lift;
//
//        public Lift(HardwareMap hardwareMap) {
//            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
//            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            lift.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class LiftUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos < 3000.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action liftUp() {
//            return new LiftUp();
//        }
//
//        public class LiftDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    lift.setPower(-0.8);
//                    initialized = true;
//                }
//
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > 100.0) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action liftDown(){
//            return new LiftDown();
//        }
//    }

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-12, -71.35, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-12, -49.41), Math.toRadians(90.00))
                .splineTo(new Vector2d(-0, -29.18), Math.toRadians(90.00));
        TrajectoryActionBuilder tab2 = tab1.fresh()
                .splineToLinearHeading(new Pose2d(-33.85, -38.45, Math.toRadians(135.00)), Math.toRadians(180.00))
                .splineToLinearHeading(new Pose2d(-42.90, -26.57, Math.toRadians(180.00)), Math.toRadians(180.00));
        TrajectoryActionBuilder tab3 = tab2.fresh()
                .splineToLinearHeading(new Pose2d(-58.81, -57.90, Math.toRadians(225)), Math.toRadians(225));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.ClawClose());


//        while (!isStopRequested() && !opModeIsActive()) {
//        }

        waitForStart();

        if (isStopRequested()) return;

        Action driveToBar = tab1.build();
        Action grabSample = tab2.build();
        Action driveToBasket = tab3.build();


        Actions.runBlocking(
                new SequentialAction(
                        driveToBar,
                        // Hang Specimen on Bar and Keep Claw Open
                        grabSample,
                        // Close Claw
                        claw.ClawClose(),
                        driveToBasket,
                        // Drop Sample in Basket
                        lift.SlidesToNet()

                )
        );
    }
}