package org.firstinspires.ftc.teamcode.TeleOp.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Armv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Liftv2;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.Robotv2;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleop Comp 3 - Basic", group="test")
@Disabled
public class TeleopComp3Basic extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private ElapsedTime runtime = new ElapsedTime();
    Robotv2 robot = null;
    boolean fieldCentric = false;

    public static double AXIAL_SCALE = 1.0;
    public static double LATERAL_SCALE = 1.0;
    public static double TURN_SCALE = 1.0;

    int leftActuatorPosition = LeftActuator.ACTUATOR_COLLAPSED;
    int rightActuatorPosition = RightActuator.ACTUATOR_COLLAPSED;

    double armPosition = Armv2.ARM_REST_POSITION;

    public static double ARM_LIFT_COMP = 0.1;
    double armLiftComp = 0;
    double armPositionFudgeFactor = 0;

    double liftPosition = Liftv2.LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    @Override
    public void init() {
        robot = new Robotv2(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));
        robot.claw.clawOpen();
        robot.wrist.WristFoldIn();
        armPosition = Armv2.ARM_REST_POSITION;
        liftPosition = Liftv2.LIFT_COLLAPSED;
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run ONCE when the driver hits STOP
     */
    @Override
    public void stop() {
        runtime.reset();
    }


    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // ===== Begin Drive Code

        //Toggle between field centric to robot centric
        if (gamepad1.options) {
            fieldCentric = !fieldCentric;
            robot.drive.resetYaw();
        }

        double gamepad1_ls_y = -gamepad1.left_stick_y;
        double gamepad_ls_x = gamepad1.left_stick_x;
        double gamepad1_rs_x = gamepad1.right_stick_x;

        gamepad1_ls_y = squaredInputWithSign(gamepad1_ls_y) * AXIAL_SCALE;
        gamepad_ls_x = squaredInputWithSign(gamepad_ls_x) * LATERAL_SCALE;
        gamepad1_rs_x = squaredInputWithSign(gamepad1_rs_x) * TURN_SCALE;

        if (fieldCentric) {
            robot.drive.moveRobotFieldCentric(gamepad1_ls_y, gamepad_ls_x, gamepad1_rs_x);
        } else {
            robot.drive.moveRobot(gamepad1_ls_y, -gamepad_ls_x, -gamepad1_rs_x);
        }
        // ===== End Drive code

        //
        if(gamepad1.right_bumper){
            robot.wrist.WristFoldIn();
            armPosition = Armv2.ARM_PICKUP_GROUND_SAMPLE_LIFT_OUT;
            liftPosition = 800;
        } else if (gamepad1.left_bumper) {
            armPosition = Armv2.ARM_CLEAR_BAR_LIFT_OUT;
            liftPosition = 800;
        } else if (gamepad1.dpad_left){
            armPosition = Armv2.ARM_REST_POSITION;
            liftPosition = 0;
        } else if (gamepad1.dpad_right){
            armPosition = Armv2.ARM_SCORE_POS;
        }

        // Claw
        if (gamepad1.b) {
            robot.claw.clawOpen();
        } else if (gamepad1.a) {
            robot.claw.clawClose();
        }

        // Wrist
        if (gamepad1.x) {
            robot.wrist.WristFoldOut();
        } else if (gamepad1.y) {
            robot.wrist.WristFoldIn();
        }



        // ===== Begin Lift code
        double liftPower = (gamepad2.right_trigger - gamepad2.left_trigger);

        liftPosition += liftPower * 300;

        if (gamepad2.right_bumper){
            liftPosition += 2800 * cycletime;
        }
        else if (gamepad2.left_bumper){
            liftPosition -= 2800 * cycletime;
        }

        /*here we check to see if the lift is trying to go higher than the maximum extension.
         *if it is, we set the variable to the max.
         */
        if (liftPosition > Liftv2.LIFT_SCORING_IN_HIGH_BASKET){
            liftPosition = Liftv2.LIFT_SCORING_IN_HIGH_BASKET;
        }
        //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
        if (liftPosition < Liftv2.LIFT_COLLAPSED){
            liftPosition = Liftv2.LIFT_COLLAPSED;
        }

        robot.lift.motor.setTargetPosition((int) (liftPosition));
        robot.lift.motor.setVelocity(3000);
        robot.lift.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ===== End lift code

        // ===== Begin Arm code
        double armFudgeFactorInput = (gamepad1.right_trigger + (-gamepad1.left_trigger));

        armPositionFudgeFactor = Armv2.FUDGE_FACTOR * armFudgeFactorInput;
        if (Math.abs(armFudgeFactorInput) > 0.05) {
            armPosition = robot.arm.motor.getCurrentPosition();
        }

        if (gamepad1.right_bumper){
            armPosition += 2800 * cycletime;
        }
        else if (gamepad1.left_bumper){
            armPosition -= 2800 * cycletime;
        }

        if (armPosition > 1000 && liftPosition > 30){
            armLiftComp = (ARM_LIFT_COMP * liftPosition);
        }
        else{
            armLiftComp = 0;
        }

           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

        // final arm position
        int armTargetPosition = (int) (armPosition + armPositionFudgeFactor + armLiftComp);
        if (armTargetPosition < Armv2.ARM_REST_POSITION){
            armTargetPosition = Armv2.ARM_REST_POSITION;
        }
            /* Here we set the target position of our arm to match the variable that was selected by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        robot.arm.motor.setTargetPosition(armTargetPosition);
        robot.arm.motor.setVelocity(500);
        robot.arm.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // ===== End Arm code


        // ===== Begin Hang Code
        {
            if (gamepad2.dpad_right) {
                robot.leftArmServo.setVertical();
                robot.rightArmServo.setVertical();
                robot.wrist.WristFoldOut();
                robot.claw.clawOpen();
                liftPosition = Liftv2.LIFT_HANG_SLIDES_POSITION;
                armPosition = Armv2.ARM_HANG_SLIDES_POSITION;
            } else if (gamepad2.dpad_left) {
                robot.leftArmServo.setHanging();
                robot.rightArmServo.setHanging();
            }

            if (gamepad2.dpad_up) {
                leftActuatorPosition = LeftActuator.ACTUATOR_UP;
                rightActuatorPosition = RightActuator.ACTUATOR_UP;
            } else if (gamepad2.dpad_down) {
                leftActuatorPosition = LeftActuator.ACTUATOR_COLLAPSED;
                rightActuatorPosition = RightActuator.ACTUATOR_COLLAPSED;
                liftPosition = Liftv2.LIFT_HANG_SLIDES_POSITION_END;
            }

            //double gamepad2_ls_x = gamepad2.left_stick_x;
            double gamepad2_ls_y = gamepad2.left_stick_y;

            //double gamepad2_rs_x = -gamepad2.right_stick_x;
            double gamepad2_rs_y = -gamepad2.right_stick_y;

            int leftActuatorCurrentPos = robot.leftActuator.motor.getCurrentPosition();
            if (gamepad2_ls_y == -1) {
                leftActuatorPosition = leftActuatorCurrentPos + 300;
            } else if (gamepad2_ls_y == 1) {
                leftActuatorPosition = leftActuatorCurrentPos - 300;
            }

            int rightActuatorCurrentPos = robot.rightActuator.motor.getCurrentPosition();

            if (gamepad2_rs_y == 1) {
                rightActuatorPosition = rightActuatorCurrentPos + 300;
            } else if (gamepad2_rs_y == -1) {
                rightActuatorPosition = rightActuatorCurrentPos - 300;
            }

            robot.leftActuator.motor.setTargetPosition(leftActuatorPosition);
            robot.leftActuator.motor.setVelocity(3000);
            robot.leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.rightActuator.motor.setTargetPosition(rightActuatorPosition);
            robot.rightActuator.motor.setVelocity(3000);
            robot.rightActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // ===== End Hang code

        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;

        // ===== Send telemetry to driver station
        dash.sendTelemetryPacket(packet);
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("code version", "parantap.5");
        telemetry.addData("wrist servo", robot.wrist.wrist.getPosition());
        telemetry.addData("claw servo", robot.claw.claw.getPosition());
        telemetry.addData("armTarget: ", robot.arm.motor.getTargetPosition());
        telemetry.addData("arm Encoder: ", robot.arm.motor.getCurrentPosition());
        telemetry.addData("lift target" , robot.lift.motor.getTargetPosition());
        telemetry.addData("lift position", robot.lift.motor.getCurrentPosition());
        telemetry.addData("FieldCentric? ", fieldCentric);
        telemetry.addData("ArmPosition", armPosition);
        telemetry.addData("ArmLiftComp", armLiftComp);
        telemetry.addData("ArmFudgeFactor", armPositionFudgeFactor);
        telemetry.addData("gamepad2.left_stick_x", gamepad2.left_stick_x);
        telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);
        telemetry.addData("gamepad2.right_stick_x", gamepad2.right_stick_x);
        telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);

    }


    private double squaredInputWithSign(double input) {
        double output = input * input;
        if (input<0){
            output = -output;
        }
        return output;
    }
}
