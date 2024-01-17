/*ckage org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel.oldmanuels;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



/**
 * "THIS IS JUST AN EXPERIMENT NOT SOMETHING IMPORTANT"
 * SAWYER PUTNAM


@TeleOp(group = "beta")
public class linearHeadlessOp extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Robot robot;
    private org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide slides;
    private org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller controller1, controller2;

    private boolean headlessMode = false;
    private boolean grip = false;

    private final double driveMultiplier = 0.70;
    private final double adjustMultiplier = 0.25;
    private double multiplier = driveMultiplier;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Robot(hardwareMap, telemetry);
        slides = new org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide(hardwareMap, telemetry);
        controller1 = new org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller(gamepad1);
        controller2 = new org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller(gamepad2);

        robot.runWithoutEncoders();
        robot.runWithBrakes();

        slides.runSlidesWithBrakes();

        while (opModeInInit()) {
            controller1.update();
            controller2.update();

            if (controller1.crossOnce()) {
                headlessMode = !headlessMode;
            }

            if (controller1.squareOnce()) {
                robot.resetHeading();
            }

            telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "yes" : "no");
            telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
            telemetry.update();
        }

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            robot.loop();

            if (controller1.crossOnce()) {
                headlessMode = !headlessMode;
            }

            if (controller1.squareOnce()) {
                robot.resetHeading();
            }

            if (controller1.circleOnce()) {
                multiplier = multiplier == driveMultiplier ? adjustMultiplier : driveMultiplier;
            }

            if (controller2.leftBumperOnce()) {
                grip = false;
            }

            if (controller2.rightBumperOnce()) {
                grip = true;
            }

            telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
            telemetry.addData("Heading (reset: square)", robot.getHeadingDegrees());
            telemetry.update();

            final double x = -Math.pow(controller1.left_stick_x, 3.0);
            final double y = Math.pow(controller1.left_stick_y, 3.0);
            double rot = Math.pow(controller1.right_trigger - controller1.left_trigger, 3.0);

            final double direction = Math.atan2(x, y) + (headlessMode ? robot.getHeading() : 0.0);
            final double magnitude = Math.min(1.0, Math.sqrt(x * x + y * y));

            double y_proc = -1 * magnitude * Math.sin(direction + Math.PI / 2.0);
            double x_proc = magnitude * Math.cos(direction + Math.PI / 2.0);


            final double driver2speed = 1.0;
            if (controller2.Triangle()) {
                y_proc = driver2speed;
                x_proc = 0.0;
            }
            if (controller2.Square()) {
                y_proc = 0.0;
                x_proc = -driver2speed;
            }
            if (controller2.Cross()) {
                y_proc = -driver2speed;
                x_proc = 0.0;
            }
            if (controller2.Circle()) {
                y_proc = 0.0;
                x_proc = driver2speed;
            }

            if (controller2.right_trigger - controller2.left_trigger != 0) {
                rot = Math.pow(controller2.right_trigger - controller2.left_trigger, 3.0) * 0.65;
            }

            final double leftFront = y_proc + x_proc + rot;
            final double leftRear = y_proc - x_proc - rot;
            final double rightFront = y_proc - x_proc + rot;
            final double rightRear = y_proc + x_proc - rot;

            robot.setMotors(leftFront, rightFront, leftRear, rightRear, multiplier);

            final double slideLeft = Math.pow(controller2.left_stick_y, 3.0);
            final double slideRight = Math.pow(controller2.left_stick_y, 3.0);
            final double slideTop = Math.pow(controller2.right_stick_y, 3.0);
            final boolean gripPower = grip;

            // Apply power to slide motors and gripper
            slides.manualHeightControl(Math.pow(controller2.left_stick_y, 3.0));
            slides.manualExtensionControl(Math.pow(controller2.right_stick_y, 3.0));
            slides.setGrip(grip);

            if (controller2.dpadUpOnce()) {
                slides.goToJunction(org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide.heights.HIGH);
            } else if (controller2.dpadRightOnce()) {
                slides.goToJunction(org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide.heights.MID);
            } else if (controller2.dpadDownOnce()) {
                slides.goToJunction(org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide.heights.LOW);
            }
        }
    }
}
*/