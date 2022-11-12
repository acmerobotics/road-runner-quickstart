package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.lift.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        final DcMotor leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        final DcMotor rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
        final Lift lift = new Lift(leftLiftMotor, rightLiftMotor);

        final Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        final Claw claw = new Claw(clawServo);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.resetEncoder();
        lift.useEncoder();

        waitForStart();
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            //if (lift != null)
            lift.useJoystick(-gamepad2.right_stick_y/2);

            if (gamepad2.a)
                claw.clawOpen();
            else if (gamepad2.b)
                claw.clawClose();

            final Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData(
                    "clawState (is open?)",
                    claw.getClawState()
            );

            final double[] liftPower = lift.getPower();
            telemetry.addData("liftPowerL", liftPower[0]);
            telemetry.addData("liftPowerR", liftPower[1]);
            telemetry.addData("lift enc L", leftLiftMotor.getCurrentPosition());
            telemetry.addData("lift enc L", rightLiftMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
