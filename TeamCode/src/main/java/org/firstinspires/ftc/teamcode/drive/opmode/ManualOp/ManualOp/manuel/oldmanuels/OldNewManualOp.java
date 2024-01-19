/**package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel.oldmanuels;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide;

@TeleOp(name = "Manual Op")
public class OldNewManualOp extends OpMode {
    // Create new robot and controller objects from the classes we defined
    private Robot robot;
    private Slide slides;
    private Controller controller1, controller2;

    // Set up some useful variables
    private boolean gripperStowed = false;  // Boolean that stows the gripper
    private boolean grip = false;                // Controls how far open and closed the gripper is
    private double multiplier = 1.0;       // Allows us to slow down the motor speed

    @Override
    public void init() {
        // Basic setup
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    @Override
    public void init_loop() {
        // Check for controller updates
        controller2.update();

        // Stow and unstow the gripper when cross on the second controller is pressed
        if (controller2.crossOnce()) {
            gripperStowed = !gripperStowed;
        }
    }

    @Override
    public void loop() {
        // We aren't using the heading features of the robot in this opmode, so we only need to update the controllers
        // This way we can save computational resources
        controller1.update();
        controller2.update();

        // Change multiplier when base driver presses circle
        if (controller1.circleOnce()) {
            multiplier = (multiplier == 1.0) ? 0.25 : 1.0;
        }

        // Stow and unstow the gripper
        if (controller2.crossOnce()) {
            gripperStowed = !gripperStowed;
        }

        // Open and close the gripper when the bumpers are pressed
        if (controller2.leftBumperOnce()) {
            grip = false;
        }
        if (controller2.rightBumperOnce()) {
            grip = true;
        }

        // Taking the input to the third power gives greater resolution at low input values without sacrificing range
        double y = -Math.pow(controller1.left_stick_y, 3.0);
        double x = Math.pow(controller1.left_stick_x, 3.0);
        double rot = Math.pow(controller1.right_trigger - controller1.left_trigger, 3.0);

        // This is Mecanum stuff, I'll do my best to explain
        /*
            To move forward and backward, you apply the same rotation to all motors
            To move side to side, each pair of diagonal motors moves together, but the motors on each side move opposite each other
            To rotate, each side must move opposite each other, but each side must move with each other

        final double leftFront = y + x + rot;
        final double leftRear = y - x + rot;
        final double rightFront = y - x - rot;
        final double rightRear = y + x - rot;

        // Set all of the drive motors
        robot.setMotors(leftFront, leftRear, rightFront, rightRear, multiplier);

        // Set up all of the arm values
        double vert = Math.pow(controller2.left_stick_y, 3.0);

        final double slideLeft = vert;
        final double slideRight = vert;
        final double slideTop = Math.pow(controller2.right_stick_y, 3.0);
        final boolean gripPower = grip;

        slides.manualHeightControl(slideLeft);
        slides.manualExtensionControl(slideTop);
        slides.setGrip(grip);
    }
}*/

