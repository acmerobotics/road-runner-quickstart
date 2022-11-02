package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
    private Lift getLift(final String motorLName, final String motorRName) {
        try {
            final DcMotor leftLiftMotor = hardwareMap.get(DcMotor.class, motorLName);
            final DcMotor rightLiftMotor = hardwareMap.get(DcMotor.class, motorRName);

            return new Lift(leftLiftMotor, rightLiftMotor);
        } catch (final IllegalArgumentException illegalArgumentException) {
            // just catch and drop, this needs to be better
            //TODO: only for comp 10/29, should not be dropping and sinking
            return null;
        }
    }

    private Claw getClaw(final String clawServoName) {
        try {
            final Servo clawServo = hardwareMap.get(Servo.class, clawServoName);
            return new Claw(clawServo);
        } catch (final IllegalArgumentException illegalArgumentException) {
            //TODO: remove after comp on 10/29, should not be sinking
            return null;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        final Lift lift = null;//getLift("leftLiftMotor", "rightLiftMotor");
        final Claw claw = getClaw("clawServo");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            if (lift != null && gamepad1.right_stick_x != 0)
                lift.useJoystick(gamepad1.right_stick_x);

            if (claw != null && gamepad1.a)
                claw.clawOpen();
            else if (claw != null && gamepad1.b)
                claw.clawClose();

            final Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.addData(
                    "clawState (is open?)",
                    claw != null ? claw.getClawState() : "no claw found"
            );

            if (lift != null) {
                final double[] liftPower = lift.getPower();
                telemetry.addData("liftPowerL", liftPower[0]);
                telemetry.addData("liftPowerR", liftPower[1]);
            }

            telemetry.update();
        }
    }
}
