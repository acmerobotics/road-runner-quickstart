package org.firstinspires.ftc.teamcode.voidvision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class defines the teleop mode for controlling the robot.
 * It uses inputs from the gamepad to drive the robot and control the linear actuator.
 */
@TeleOp(name="practice teleop", group="Pushbot")
public class practiceteleop extends Auto_Util {

    // Hardware map and elapsed time instance
    practicehwmap robot = new practicehwmap();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables for motor power
    static double turnPower;
    static double fwdBackPower;
    static double strafePower;
    static double lbPower;
    static double lfPower;
    static double rbPower;
    static double rfPower;
    static double lAPower;  // Linear actuator power

    @Override
    public void runOpMode() {
        // Initialize robot hardware and autonomous utilities
        robot.init(hardwareMap);
        initAuto();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the driver to press start
        waitForStart();

        // Main control loop
        while (opModeIsActive()) {

            // Gamepad inputs for driving
            fwdBackPower = -gamepad1.left_stick_y;  // Forward and backward movement
            strafePower = -gamepad1.left_stick_x;   // Strafing movement (left and right)
            turnPower = -gamepad1.right_stick_x;    // Turning left and right
            lAPower = -gamepad2.left_stick_y;       // Control for the linear actuator (using gamepad2)

            // Calculate motor power for mecanum drive
            lfPower = (fwdBackPower - turnPower - strafePower);
            rfPower = (fwdBackPower + turnPower + strafePower);
            lbPower = (fwdBackPower - turnPower + strafePower);
            rbPower = (fwdBackPower + turnPower - strafePower);

            // Set motor powers to the robot's drive motors
            robot.leftfrontDrive.setPower(lfPower);
            robot.rightfrontDrive.setPower(rfPower);
            robot.leftbackDrive.setPower(lbPower);
            robot.rightbackDrive.setPower(rbPower);

            // Set power for the linear actuator
            robot.linearActuator.setPower(lAPower);

            telemetry.update();
        }
    }
}
