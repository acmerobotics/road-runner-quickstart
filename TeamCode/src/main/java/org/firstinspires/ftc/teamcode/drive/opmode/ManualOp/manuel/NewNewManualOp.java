package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Slide;

@Config
@TeleOp (name = "WITH CLAW WORK")

public class NewNewManualOp extends OpMode {

    private Robot robot;
    // Create two new Controller objects, one for each gamepad

    private Telemetry telemetry;
    private Controller base, arm;

    private Slide slides;
    // Set up some useful variables
    private boolean headlessMode = false;   // Allows us to toggle headless mode on and off
                                            // Controls how far open or closed the gripper is
    private final double multiplier = 1;
    private final double mainMultiplier = 0.7;
    private final double adjustMultiplier = 0.25;
   // private double multiplier = mainMultiplier;// Allows us to scale down the motor speed

    private int wristInput,gripInput;

    // This code will run when the init button is pressed on the Driver Hub
    @Override
    public void init() {
        // Basic setup
        robot = new Robot(hardwareMap, telemetry);  // Initialize our robot class
        robot.runWithoutEncoders();                   // Tell our drive motors to use encoders
        robot.runSlideWithoutEncoders();            // Tell our slide motors not to use encoders
        base = new Controller(gamepad1);     // Initialize controller1
        arm = new Controller(gamepad2);     // Initialize controller2
        slides = new Slide(hardwareMap,telemetry); //initialize our slide
        robot.runWithBrakes();  // Tell our driv motors to use brakes
    }

    // This code will after the init block and will loop until the start button is pressed
    @Override
    public void init_loop() {
        // Check for controller updates
        base.update();
        arm.update();

        // Toggle headless mode when cross is pressed on base driver's controller
        if (base.crossOnce()) {
            headlessMode = ! headlessMode;
        }

        // Reset heading when base driver presses square
        if (base.squareOnce()) {
            robot.resetHeading();
        }

        // Add some telemetry information for convenience
        telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "yes" : "no");
        telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
        telemetry.addData("clawthing", wristInput);
        telemetry.update();
    }

    @Override
    public void loop() {

        /**
         * base Controls
         */
        double y = base.left_stick_y;
                // this is an exponetial curve lowkey think linear is better but if we want ex we can comment it back in
                //-Math.pow(base.left_stick_y, 3.0);
        double x = base.left_stick_x;
                //Math.pow(base.left_stick_x, 3.0);
        double rot = base.right_trigger - base.left_trigger;
              //  Math.pow(base.right_trigger - base.left_trigger, 3.0);

        // This is Mecanum stuff, I'll do my best to explain
        /*
            To move forward and backward, you apply the same rotation to all motors
            To move side to side, each pair of diagonal motors moves together, but the motors on each side move opposite each other
            To rotate, each side must move opposite each other, but each side must move with each other
         */

        //TODO I'll need to work on motor rotations
        final double leftFront = y + x + rot;
        final double leftRear = y - x + rot;
        final double rightFront = y - x - rot;
        final double rightRear = y + x - rot;

        // Set all of the drive motors
        robot.setMotors(leftFront, leftRear, rightFront, rightRear, multiplier);

        /**
         * Arm Conrols
         */

        // Set up the arm slides
        double vert = Math.pow(arm.left_stick_y, 3.0);

        final double slide = vert;

        slides.manualHeightControl(slide);





        /**
         * toggle the wrist action the left bumper
         *
         * this could be placed in headless op
         *
         * if we press the left bumper then a counter is increased by 1
         * if the counter is even then the claw is open if odd the wrist is in the intake position
         *
         * there is a potential of also combining this action with the opening and closing of the claw
         *
         * check for d bounce if there is debounce then go in controller and create a once function
         */
        telemetry.addData("wrist", wristInput);
        telemetry.addData("claw", gripInput);

        telemetry.update();
        //TODO hope this works if not then curl up in a ball and cry also check for d bounce problems


        if (base.leftBumper()) {

            wristInput++;
        }

        robot.setWrist(wristInput % 2 == 0);

        // bassically the same thing for the opening
        if (base.rightBumper()){
            gripInput++;
        }
        robot.setGrip(gripInput % 2 == 0);


    }
}
