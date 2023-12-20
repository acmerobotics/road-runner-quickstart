package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//imports from the Mecanum website
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(group = "a")
public class BasicTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.BLUE);
        //TrajectoryBook book = new TrajectoryBook(drive, extras);

        int IMUReset = 0;
        int elevatorBottomThreshold = 200;
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double speedMultiplier;

        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_right_bumper_pressed = false;
        boolean gp2_right_trigger_pressed = false;
        boolean gp2_right_stick_y_neg_pressed = false;
        boolean gp2_right_stick_y_pos_pressed = false;
        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;
        boolean gp2_y_pressed = false;
        boolean gp1_y_pressed = false;
        boolean gp1_a_pressed = false;
        boolean gp2_x_pressed = false;

        boolean elevatorStopped = true;
        boolean elevatorBottom = true;

        double elevMultMin = 0.5;
        double elevMult = 0;
        double elevHeightMax = 800;
        double slope;
        double elevatorEncoderCounts;

        double currentAmpsLeft;
        double currentAmpsRight;
        double maxAmps = 0;
        int numDangerAmps = 0;

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        waitForStart();

        while (!isStopRequested())
        {


            currentAmpsLeft = extras.elevatorLeft.getCurrent(CurrentUnit.AMPS);
            currentAmpsRight = extras.elevatorRight.getCurrent(CurrentUnit.AMPS);

            if (currentAmpsLeft > maxAmps)
            {
                maxAmps = currentAmpsLeft;
            }
            else if (currentAmpsRight > maxAmps)
            {
                maxAmps = currentAmpsRight;
            }

            if (currentAmpsLeft >= 7 || currentAmpsRight >= 7)
            {
                numDangerAmps += 1;

                telemetry.addLine("WARNING: HIGH AMPS");

                if(numDangerAmps >= 10)
                {
                    // in future, ONLY stop arm motion
                    stop();
                }
            }
            else
            {
                numDangerAmps = 0;
            }

            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevatorLeft.getCurrentPosition() + extras.elevatorRight.getCurrentPosition()) / 2;
            elevMult = slope * elevatorEncoderCounts + 1;

            if (gamepad1.right_bumper)
            {
                speedMultiplier = 0.6 * elevMult;
            }
            else if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.4 * elevMult;
            }
            else
            {
                speedMultiplier = 0.75 * elevMult;
            }

            adjustedAngle = extras.adjustAngleForDriverPosition(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            //stickForward = -gamepad1.left_stick_y * speedMultiplier;
            //stickSideways = -gamepad1.left_stick_x * speedMultiplier;
            //stickSidewaysRotated = (stickSideways * Math.cos(adjustedAngle)) - (stickForward * Math.sin(adjustedAngle));
            //stickForwardRotated = (stickSideways * Math.sin(adjustedAngle)) + (stickForward * Math.cos(adjustedAngle));
            stickSideways = gamepad1.left_stick_x * speedMultiplier;
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(-adjustedAngle)) - (stickForward * Math.sin(-adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(-adjustedAngle)) + (stickForward * Math.cos(-adjustedAngle));

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            stickSidewaysRotated,
                            stickForwardRotated
                            //stickForwardRotated,
                            //stickSidewaysRotated
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            float elevatorStick = gamepad2.left_stick_y;

            if ((elevatorStick >= 0) && !elevatorStopped) {
                extras.elevatorLeft.setPower(0);
                extras.elevatorRight.setPower(0);
                elevatorStopped = true;

                int elevPos1 = extras.elevatorLeft.getCurrentPosition();
                extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevatorLeft.setTargetPosition(elevPos1);
                extras.elevatorRight.setTargetPosition(elevPos1);
                extras.elevatorLeft.setPower(1.0);
                extras.elevatorRight.setPower(1.0);

            } else if (elevatorStick < 0) {
                //moves the elevator with the left joystick
                extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extras.elevatorLeft.setPower(-elevatorStick*0.2);
                extras.elevatorRight.setPower(-elevatorStick*0.2);
                elevatorStopped = false;
            }
            //disables the outdated elevator code
            if(false) {
                // MANUAL ELEVATOR CONTROL- gamepad 2
                if (extras.elevatorLeft.getCurrentPosition() > elevatorBottomThreshold) {
                    elevatorBottom = false;
                }
                // stop if the limit switch is pressed
                if ((extras.elevatorLeft.getCurrentPosition() < elevatorBottomThreshold) && (elevatorBottom == false)) {
                    //extras.elevatorLeft.setPower(0);
                    //extras.elevatorRight.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevatorLeft.getCurrentPosition();
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    //extras.elevatorLeft.setTargetPosition(elevPos1);
                    //extras.elevatorRight.setTargetPosition(elevPos1);
                    extras.elevatorLeft.setTargetPosition(0);
                    extras.elevatorRight.setTargetPosition(0);
                    extras.elevatorLeft.setPower(0.2);
                    extras.elevatorRight.setPower(0.2);
                    elevatorBottom = true;

                    // it's OK to move up if the limit switch is pressed
                /*if(elevatorStick < 0)
                {
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevatorLeft.setPower(-elevatorStick);
                    extras.elevatorRight.setPower(-elevatorStick);
                    elevatorStopped = false;
                }*/
                }

                // don't go above the max height
                else if ((extras.elevatorLeft.getCurrentPosition() > elevHeightMax) && (elevatorStick < 0)) {
                    extras.elevatorLeft.setPower(0);
                    extras.elevatorRight.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevatorLeft.getCurrentPosition();
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorLeft.setTargetPosition(elevPos1);
                    extras.elevatorRight.setTargetPosition(elevPos1);
                    extras.elevatorLeft.setPower(1.0);
                    extras.elevatorRight.setPower(1.0);
                }

                // don't go too low if elbow is extended
                else if ((extras.elevatorLeft.getCurrentPosition() < 200) && (elevatorStick > 0) && (extras.elbowPosition != ExtraOpModeFunctions.ElbowPosition.EXTEND)) {
                    extras.elevatorLeft.setPower(0);
                    extras.elevatorRight.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevatorLeft.getCurrentPosition();
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorLeft.setTargetPosition(elevPos1);
                    extras.elevatorRight.setTargetPosition(elevPos1);
                    extras.elevatorLeft.setPower(1.0);
                    extras.elevatorRight.setPower(1.0);
                } else {
                    // If stick is not moved, only set power to 0 once
                    if ((elevatorStick == 0) && !elevatorStopped) {
                        extras.elevatorLeft.setPower(0);
                        extras.elevatorRight.setPower(0);
                        elevatorStopped = true;

                        int elevPos1 = extras.elevatorLeft.getCurrentPosition();
                        extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        extras.elevatorLeft.setTargetPosition(elevPos1);
                        extras.elevatorRight.setTargetPosition(elevPos1);
                        extras.elevatorLeft.setPower(1.0);
                        extras.elevatorRight.setPower(1.0);

                    } else if (elevatorStick != 0) {
                        extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        extras.elevatorLeft.setPower(-elevatorStick);
                        extras.elevatorRight.setPower(-elevatorStick);
                        elevatorStopped = false;
                    }
                }
            }
            /*


            // Elevator to top with right stick up
            if (gamepad2.right_stick_y < 0)
            {
                extras.elevatorHigh();
            }

            // Elevator to bottom with right sitck down
            if ((gamepad2.right_stick_y > 0)  && (extras.wristPosition == ExtraOpModeFunctions.WristPosition.MIDDLE))
            {
                extras.elevatorGround();
            }
            */

            // RESET IMU
            if ((gamepad1.back) && (gamepad1.b))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();
                drive.imu.resetYaw();
            }

            /*
            // Init Elevator
            if ((gamepad2.back) && (gamepad2.y))
            {
                gp2_y_pressed = true;
            }
            else if (!gamepad2.y && gp2_y_pressed)
            {
                //extras.initElevator();
                gp2_y_pressed = false;

            }
            */

            // Releases lower pixel
            if (gamepad2.a)
            {
                gp2_a_pressed = true;
            }
            else if (!gamepad2.a && gp2_a_pressed)
            {
                extras.lowerClawRelease();
                gp2_a_pressed = false;
            }


            // Releases upper pixel
            if (gamepad2.y)
            {
                gp2_y_pressed = true;
            }
            else if (!gamepad2.y && gp2_y_pressed)
            {
                extras.upperClawRelease();
                gp2_y_pressed = false;
            }

            //debug - grab a pixel with upper claw
            //change to extend
            if (gamepad2.dpad_up)
            {
                gp2_dpad_up_pressed = true;
            }
            else if (!gamepad2.dpad_up && gp2_dpad_up_pressed)
            {
                extras.elbowExtend();
                gp2_dpad_up_pressed = false;
            }

            //debug - grab a pixel with lower claw
            //change to lowering the elbow
            if (gamepad2.dpad_down)
            {
                gp2_dpad_down_pressed = true;
            }
            else if (!gamepad2.dpad_down && gp2_dpad_down_pressed)
            {
                extras.elbowRetract();
                gp2_dpad_down_pressed = false;
            }

            //positionIn
            if (gamepad2.x)
            {
                extras.armStateMachine(ExtraOpModeFunctions.ArmStateMachineAction.XPRESSED);
            }
            else if (gamepad2.b && !gamepad2.start)
            {
                extras.armStateMachine(ExtraOpModeFunctions.ArmStateMachineAction.BPRESSED);
            }
            else
            {
                extras.armStateMachine(ExtraOpModeFunctions.ArmStateMachineAction.IDLE);
            }

            //-----------------------
            //Buttons for controller 1/gamepad 1
            //Press Y to extend the lift, and press A to lift the robot up

            //extend lift
            if (gamepad1.y)
            {
                extras.liftExtend();
            }
            //retract lift
            else if (gamepad1.a && !gamepad1.start)
            {
                extras.liftRetract();
            }
            //turn lift off
            else
            {
                extras.liftOff();
            }

            //----------------------

            if (gamepad2.left_bumper)
            {
                extras.intakeReverse();
            }
            else if (gamepad2.left_trigger > 0)
            {
                extras.intakeForward();
            }
            else
            {
                extras.intakeOff();
            }

            extras.setLeds(getRuntime());

            telemetry.addData("step", extras.step);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading.real);
            telemetry.addData("Pid Left", extras.elevatorLeft.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.addData("Pid Right", extras.elevatorRight.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION));
            telemetry.addData("Elevator Left encoder counts: ", extras.elevatorLeft.getCurrentPosition());
            telemetry.addData("Elevator Right encoder counts: ", extras.elevatorRight.getCurrentPosition());
            telemetry.addData("Elevator limit: ", extras.elevatorLimit.isPressed());
            telemetry.addData("Elevator stopped? ", elevatorStopped);
            telemetry.addData("lift position", extras.lift.getCurrentPosition());
            telemetry.addLine();

            telemetry.addData("Elevator Left current: ", currentAmpsLeft);
            telemetry.addData("Elevator Right current: ", currentAmpsRight);
            telemetry.addData("Max amps: ", maxAmps);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}