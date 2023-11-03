package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="LinearActuatorTest", group="SCC")
public class LinearActuatorTest extends LinearOpMode {
    static final int    MIN_MOTOR_ENCODER_POS    =   150;
    static final int    MAX_MOTOR_ENCODER_POS    =   13200;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor linearActuatorMotor = null;
    private TouchSensor linearActuatorTouchSensor;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup hardware map
        linearActuatorMotor  = hardwareMap.get(DcMotor.class, "linear_actuator_motor");
        linearActuatorTouchSensor = hardwareMap.get(TouchSensor.class, "linear_actuator_touch_sensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        linearActuatorMotor.setDirection(DcMotor.Direction.FORWARD);
        linearActuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearActuatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearActuatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double motorPower;

            motorPower  = -gamepad1.left_stick_y;

            if (linearActuatorTouchSensor.isPressed()) {
                // Send calculated power to wheels
                linearActuatorMotor.setPower(0.0);
            } else {
                // Are we going up and has the upper limit been reached?
                if (motorPower > 0.01 && linearActuatorMotor.getCurrentPosition() <= MAX_MOTOR_ENCODER_POS) {
                    linearActuatorMotor.setPower(motorPower);
                } else if (motorPower < -0.01 && linearActuatorMotor.getCurrentPosition() >= MIN_MOTOR_ENCODER_POS) {
                    linearActuatorMotor.setPower(motorPower);
                } else {
                    linearActuatorMotor.setPower(0.0);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Motor Power: %.2f", motorPower);
            telemetry.addData("Current Motor Position", "%7d", linearActuatorMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}