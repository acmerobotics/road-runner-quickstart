/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.oldCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class Controller extends OpMode {
    RobotHardware robot;
    ElapsedTime runtime;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap, false);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        telemetry.addData("Status", "Running");


    }


    @Override
    public void loop() {
        driveControl();
        liftControl();
        turretControl();
        extensionControl();
        manualControl();
    }

    public int liftPos = 0;
    public int liftDropPos = liftPos - 200;

    private void driveControl() {
        double scale = 0.3;
        if (gamepad1.left_bumper) {
            gamepad1.rumble(500);
            scale = 0.6;
        } else if (gamepad1.left_trigger > 0.5) {
            scale = 0.1;
        }

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.driveTrain.startMove(drive, strafe, turn, scale);

        robot.driveTrain.telemetryUpdate(telemetry);
    }

    private void liftControl() {
        if (gamepad1.y) {
            robot.lift.HighJunction();
            liftPos = robot.lift.LIFT_HIGH_POS;
        }
        if (gamepad1.x) {
            robot.lift.MiddleJunction();
            liftPos = robot.lift.LIFT_MID_POS;
        }
        if (gamepad1.b) {
            robot.lift.LowJunction();
            liftPos = robot.lift.LIFT_LOW_POS;
        }
        if (gamepad1.a) {
            robot.lift.Intake();
            liftPos = robot.lift.LIFT_INTAKE_POS;
        }
        if (gamepad1.left_trigger > 0.5 && gamepad1.b) {
            robot.lift.HOVER();
            liftPos = robot.lift.LIFT_HOVER_POS;
        }


        if (gamepad1.right_trigger > 0.5 && robot.lift.motorLiftR.getCurrentPosition() > 300) {
            if (robot.lift.motorLiftR.getCurrentPosition() > 2800 && robot.lift.motorLiftR.getCurrentPosition() < 3200) {
                robot.lift.motorLiftL.setTargetPosition(2400);
                robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftL.setPower(0.5);
                robot.lift.motorLiftR.setTargetPosition(2400);
                robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftR.setPower(0.5);
            }
            if (robot.lift.motorLiftR.getCurrentPosition() > 1800 && robot.lift.motorLiftR.getCurrentPosition() < 2200) {
                robot.lift.motorLiftL.setTargetPosition(1200);
                robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftL.setPower(0.5);
                robot.lift.motorLiftR.setTargetPosition(1200);
                robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftR.setPower(0.5);
            }
            if (robot.lift.motorLiftR.getCurrentPosition() > 1100 && robot.lift.motorLiftR.getCurrentPosition() < 1400) {
                robot.lift.motorLiftL.setTargetPosition(950);
                robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftL.setPower(0.5);
                robot.lift.motorLiftR.setTargetPosition(950);
                robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.motorLiftR.setPower(0.5);
            }
        } else {
            robot.lift.motorLiftL.setTargetPosition(liftPos);
            robot.lift.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.motorLiftL.setPower(1);
            robot.lift.motorLiftR.setTargetPosition(liftPos);
            robot.lift.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.motorLiftR.setPower(1);
        }

        if (gamepad1.right_bumper) {
                robot.lift.servoClaw.setPosition(robot.lift.CLAW_OPEN_POS);
        } else {
                robot.lift.servoClaw.setPosition(robot.lift.CLAW_CLOSE_POS);
            }



            robot.lift.run();
            telemetry.addData("LEFT LIFT pos", robot.lift.motorLiftL.getCurrentPosition());
            telemetry.addData("RIGHT LIFT pos", robot.lift.motorLiftR.getCurrentPosition());
            telemetry.addData("TURRET pos", robot.lift.motorTurret.getCurrentPosition());
//        telemetry.addData("Distance cm", robot.distanceSensor.getDistance());



        telemetry.addData("state", robot.lift.currentState);
        }

        public void turretControl () {
            if (gamepad1.dpad_left) {
                robot.lift.TurretLeft();
            }
            if (gamepad1.dpad_right) {
                robot.lift.TurretRight();
            }
            if (gamepad1.dpad_up) {
                robot.lift.Turret180Cycle();
            }
            if (gamepad1.dpad_down) {
                robot.lift.Turret0();
            }
            if (gamepad1.dpad_right && gamepad1.left_trigger > 0.5) {
                robot.lift.TurretRightIntake();
            }
            if (gamepad1.dpad_left && gamepad1.left_trigger > 0.5) {
                robot.lift.TurretLeftIntake();
            }
            if (gamepad1.left_trigger > 0.5 && gamepad1.dpad_up) {
                robot.lift.Turret180();
            }
        }



        public void extensionControl () {
            if (robot.lift.currentState == RobotHardware.Lift.States.INTAKE && gamepad1.right_trigger > 0.5) {
                robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_OUT_POS);
            } else if (robot.lift.currentState == RobotHardware.Lift.States.INTAKE) {
                robot.lift.servoExtension.setPosition(robot.lift.EXTENSION_INTAKE_POS);
            }
        }

        public void manualControl () {

        if (gamepad2.b) {
            robot.lift.currentState = RobotHardware.Lift.States.MANUAL_MODE;
        }
        if (robot.lift.currentState == RobotHardware.Lift.States.MANUAL_MODE) {
            if (gamepad2.dpad_down) {
                robot.lift.motorLiftR.setPower(-0.5);
                robot.lift.motorLiftL.setPower(-0.5);
            } else if (gamepad2.dpad_up) {
                robot.lift.motorLiftR.setPower(0.5);
                robot.lift.motorLiftL.setPower(0.5);
            } else {
                robot.lift.motorLiftR.setPower(0);
                robot.lift.motorLiftL.setPower(0);
            }

            if (gamepad2.dpad_right) {
                robot.lift.motorTurret.setPower(-0.5);
            } else if (gamepad2.dpad_left) {
                robot.lift.motorTurret.setPower(0.5);
            } else {
                robot.lift.motorTurret.setPower(0);
            }
        }
        }

//            if (gamepad2.a) {
//                robot.lift.servoExtension.setPosition(0.6);
//            } else if (gamepad2.b) {
//                robot.lift.servoExtension.setPosition(0.5);
//            } else if (gamepad2.x) {
//                robot.lift.servoExtension.setPosition(0.4);
//            }
//            }



        /*
         * Code to run ONCE after the driver hits STOP
         */


    }
    