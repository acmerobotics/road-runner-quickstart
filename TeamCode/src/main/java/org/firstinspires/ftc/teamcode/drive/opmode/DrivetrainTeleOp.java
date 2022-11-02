package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;

@TeleOp(group = "drive")
public class DrivetrainTeleOp extends LinearOpMode {
    /*
    This maps angles to directions which we use to decide which direction the drivetrain should go
     */
    private final HashMap<Double, Drivetrain.Direction> angleDirectionMap = new HashMap<>();

    private final static boolean USE_ALTERNATE_DRIVE_BEHAVIOR = false;

    // These are all of the directions for each of the angles
    public DrivetrainTeleOp() {
        super();
        angleDirectionMap.put(22.5, Drivetrain.Direction.RIGHT);
        angleDirectionMap.put(67.5, Drivetrain.Direction.FORWARD_RIGHT);
        angleDirectionMap.put(112.5, Drivetrain.Direction.FORWARD);
        angleDirectionMap.put(157.5, Drivetrain.Direction.FORWARD_LEFT);
        angleDirectionMap.put(202.5, Drivetrain.Direction.LEFT);
        angleDirectionMap.put(247.5, Drivetrain.Direction.BACKWARD_LEFT);
        angleDirectionMap.put(292.5, Drivetrain.Direction.BACKWARD);
        angleDirectionMap.put(337.5, Drivetrain.Direction.BACKWARD_RIGHT);
        angleDirectionMap.put(360.0, Drivetrain.Direction.RIGHT);
    }
    //It takes the x and y coordinate of where the joystick is, and gives back an angle
    private double getAngle(final double x, final double y) {
        final double angle;
        return (angle = Math.toDegrees(Math.atan2(x, y))) < 0 ? (angle + 360) : angle;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //We are getting a a reference for the 4 motors from the hardware maps
        final DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        final DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        final DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        final DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        //Make a new dt with our 4 motors
        final Drivetrain drivetrain = new Drivetrain(
                leftBackMotor, leftFrontMotor, rightBackMotor, rightFrontMotor
        );
        //Wait for TeleOp to start
        waitForStart();
        while (!isStopRequested()) {
            final double leftStickX = gamepad1.left_stick_x;
            final double leftStickY = gamepad1.left_stick_y;
            final double rightStickX = gamepad1.right_stick_x;

            if (!USE_ALTERNATE_DRIVE_BEHAVIOR) {
                //Get the angle of the left joystick
                final double leftJoystickAngle = getAngle(leftStickX, leftStickY);

                //Get the minimum angle that the current joystick angle is less than
                double minAngle = 360d;
                for (final Map.Entry<Double, Drivetrain.Direction> entry : angleDirectionMap.entrySet()) {
                    final double entryAngle = entry.getKey();
                    if (leftJoystickAngle < entryAngle && entryAngle < minAngle)
                        minAngle = entryAngle;
                }
                //Get the direction according to the minimum angle
                final Drivetrain.Direction direction = angleDirectionMap.get(minAngle);
                //Find speed from x and y components
                final double speed = Math.sqrt((leftStickX * leftStickX) + (leftStickY * leftStickY));
                //Tell the dt to drive according to direction and speed
                drivetrain.driveDirection(direction, speed);

                telemetry.addData("PWR_LFM", leftFrontMotor.getPower());
                telemetry.addData("PWR_LBM", leftBackMotor.getPower());
                telemetry.addData("PWR_RFM", rightFrontMotor.getPower());
                telemetry.addData("PWR_RBM", rightBackMotor.getPower());
                telemetry.update();
            } else {
                drivetrain.driveRobotCentricFromJoysticks(leftStickY, leftStickX, rightStickX);
            }
        }
    }
}
