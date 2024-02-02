/**package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel.oldmanuels;

/**
 * Mecanum teleop (with an optional headless mode)
 * * Left stick controls x/y translation.
 * * Triggers control rotation about the z axis
 * * When headless mode is enabled (press "cross"), translation direction
 * becomes relative to the field as opposed to the robot. You can
 * reset the forward heading by pressing "square".

@TeleOp(name = "HeadlessOpTEST")

public class HeadlessOp extends OpMode {

    // Create new Robot object named robot
    private Robot robot;
    // Create two new Controlle-r objects, one for each gamepad
    private Controller controller1, controller2;

    // Set up some useful variables
    private boolean headlessMode = false;   // Allows us to toggle headless mode on and off
    private boolean grip = false;           // Controls how far open or closed the gripper is

    private final double mainMultiplier = 0.7;
    private final double adjustMultiplier = 0.25;
    private double multiplier = mainMultiplier;        // Allows us to scale down the motor speed

    // This code will run when the init button is pressed on the Driver Hub
    @Override
    public void init() {
        // Basic setup
        robot = new Robot(hardwareMap, telemetry);  // Initialize our robot class
        robot.runWithoutEncoders();                   // Tell our drive motors to use encoders
        robot.runSlideWithoutEncoders();            // Tell our slide motors not to use encoders
        controller1 = new Controller(gamepad1);     // Initialize controller1
        controller2 = new Controller(gamepad2);     // Initialize controller2

        robot.runWithBrakes();  // Tell our driv motors to use brakes
    }

    // This code will after the init block and will loop until the start button is pressed
    @Override
    public void init_loop() {
        // Check for controller updates
        controller1.update();
        controller2.update();

        // Toggle headless mode when cross is pressed on base driver's controller
        if (controller1.crossOnce()) {
            headlessMode = ! headlessMode;
        }

        // Reset heading when base driver presses square
        if (controller1.squareOnce()) {
            robot.resetHeading();
        }

        // Add some telemetry information for convenience
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "yes" : "no");
        telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
        telemetry.update();
    }

    // This code will run once the start button is pressed
    @Override
    public void loop() {
        // Check controllers for updates
        controller1.update();
        controller2.update();
        // Update robot heading
        robot.loop();

        // Reset heading when base driver presses square
        if (controller1.squareOnce()) {
            robot.resetHeading();
        }

        // Toggle headless when base driver presses cross
        if (controller1.crossOnce()) {
            headlessMode = !headlessMode;
        }

        // Change drive multiplier when base driver presses circle
        if (controller1.circleOnce()) {
            multiplier = multiplier == mainMultiplier ? adjustMultiplier : mainMultiplier;
        }

        // Set grippers to open when the left trigger is pressed
        if (controller2.leftTriggerOnce()) {
            grip = false;
        }
        // Set grippers to close when the right trigger is pressed
        if (controller2.rightTriggerOnce()) {
            grip = true;
        }

        // Add telemetry for convenience
        telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
        telemetry.addData("Heading (reset: square)", robot.getHeadingDegrees());
        telemetry.update();

        // This large if/else block allows the arm driver to take control of the base to make fine
        // adjustments. This cuts down on communication slowdowns in some cases
        if (controller2.Circle() || controller2.Cross() ||
                controller2.Square() || controller2.Triangle()) {
            if (controller2.Triangle()) {
                robot.setMotors(0.75f, 0.75f, 0.75f,
                        0.75f, 1);
            }
            if (controller2.Cross()) {
                robot.setMotors(-0.75f, -0.75f, -0.75f,
                        -0.75f, 1);
            }
            if (controller2.Square()) {
                robot.setMotors(-0.75f, 0.75f, 0.75f,
                        -0.75f, 1);
            }
            if (controller2.Circle()) {
                robot.setMotors(0.75f, -0.75f, -0.75f,
                        0.75f, 1);
            }
        }
        else if (controller2.dpadDown() || controller2.dpadUp() ||
                controller2.dpadLeft() || controller2.dpadRight()) {
            if (controller2.dpadUp()) {
                robot.setMotors(0.1f, 0.1f, 0.1f,
                        0.1f, 1);
            }
            if (controller2.dpadDown()) {
                robot.setMotors(-0.1f, -0.1f, -0.1f,
                        -0.1f, 1);
            }
            if (controller2.dpadLeft()) {
                robot.setMotors(-0.1f, 0.1f, 0.1f,
                        -0.1f, 1);
            }
            if (controller2.dpadRight()) {
                robot.setMotors(0.1f, -0.1f, -0.1f,
                        0.11f, 1);
            }
        }
        else if (controller2.rightBumper()) {
            robot.setMotors(0.25, 0.25, -0.25, -0.25, 1);
        }
        else if (controller2.leftBumper()) {
            robot.setMotors(-0.3f, -0.3f, 0.3f, 0.3f, 1);
        }
        // When the arm driver isn't overriding base controls, this code controls the motor
        else {
            // Get input from the base driver's left stick and take it to
            // the third power to increase low-range resolution
            final double x = -Math.pow(controller1.left_stick_x, 3.0);
            final double y = Math.pow(controller1.left_stick_y, 3.0);
            final double rotation = Math.pow(controller1.right_trigger - controller1.left_trigger, 3.0);

            // get direction as the counterclockwise angle from the x-axis to point (x, y)
            // then compensate for reference angle by adding current heading
            final double direction = Math.atan2(x, y) + (headlessMode ? robot.getHeading() : 0.0);
            // determine speed using pythagorean theorem
            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

            // another attempt at making sense of this headless stuff
            // basically splitting apart everything into basic x and y again, but with direction,
            // then doing it like normal? idk man it's 3am and i'm confused
            final double y_proc = -1 * speed * Math.sin(direction + Math.PI / 2.0);
            final double x_proc = speed * Math.cos(direction + Math.PI / 2.0);

            // This is Mecanum stuff, I'll do my best to explain
        /*
            To move forward and backward, you apply the same rotation to all motors
            To move side to side, each pair of diagonal motors moves together,
            but the motors on each side move opposite each other
            The rotation is different from normal for some strange reason. I have no clue why. Sorry.

            final double leftFront = y_proc + x_proc + rotation;
            final double leftRear = y_proc - x_proc - rotation;
            final double rightFront = y_proc - x_proc + rotation;
            final double rightRear = y_proc + x_proc - rotation;

            // Set all of the drive motors
            robot.setMotors(leftFront, rightFront, leftRear, rightRear, multiplier);
        }

        // Set up all of the arm values
        final double slideLeft = Math.pow(controller2.left_stick_y, 3.0);
        final double slideRight = Math.pow(controller2.left_stick_y, 3.0);
        final double slideTop = Math.pow(controller2.right_stick_y, 3.0);
        final boolean gripPower = grip;

        // Apply power to slide motors and gripper
        robot.setSlideMotors(slideLeft, slideRight, slideTop);
        robot.setGrip(gripPower);

    }
}*/