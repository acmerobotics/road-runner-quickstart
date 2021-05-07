//     /* Copyright (c) 2017 FIRST. All rights reserved.
//      *
//      * Redistribution and use in source and binary forms, with or without modification,
//      * are permitted (subject to the limitations in the disclaimer below) provided that
//      * the following conditions are met:
//      *
//      * Redistributions of source code must retain the above copyright notice, this list
//      * of conditions and the following disclaimer.
//      *
//      * Redistributions in binary form must reproduce the above copyright notice, this
//      * list of conditions and the following disclaimer in the documentation and/or
//      * other materials provided with the distribution.
//      *
//      * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//      * promote products derived from this software without specific prior written permission.
//      *
//      * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//      * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//      * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//      * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//      * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//      * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//      * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//      * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//      * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//      * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//      * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//      */

//     package org.firstinspires.ftc.teamcode.OldCode;

//     import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//     import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//     import com.qualcomm.robotcore.hardware.DcMotor;
//     import com.qualcomm.robotcore.hardware.DigitalChannel;
//     import com.qualcomm.robotcore.util.ElapsedTime;
//     import com.qualcomm.robotcore.util.Range;

//     /**
//      * This file contains an example of an iterative (Non-Linear) "OpMode".
//      * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
//      * The names of OpModes appear on the menu of the FTC Driver Station.
//      * When an selection is made from the menu, the corresponding OpMode
//      * class is instantiated on the Robot Controller and executed.
//      *
//      * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
//      * It includes all the skeletal structure that all iterative OpModes contain.
//      *
//      * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
//      * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
//      */

//     @TeleOp(name="UltimateGoalTeleOpTest", group="UltimateGoal")

//     public class UltimateGoalTeleOpWobble extends OpMode
//     {
//         /**
//          * Hardware class implementation
//          */
//         UltimateGoalHardwareTest        mRobot;

//         /**
//          * Quad robot drive.
//          */
//         RobotDrive               mDrive;



//         private TelemetryLogger  mLogger;

//         /**
//          * Timer to monitor duration of the calibrated operations.
//          */
//         private ElapsedTime         mTimer;



//         @Override
//         public void init()
//         {
//             initLogging();
//             mLogger.info("Initializing");

//             mTimer = new ElapsedTime();
//             mRobot = new UltimateGoalHardwareTest(this, mLogger);
//             mRobot.init();
//             //mRobot.initGyro();

//             mDrive = mRobot.getRobotDrive();

//             mDrive.getFirstLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//             mDrive.getFirstRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//             mDrive.getLastLeftDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//             mDrive.getLastRightDrive().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//             // In teleop mode, the drive will not be operated using encoders - set it to non-encoder mode.
//             mDrive.prepareToRotate();
            
//             mRobot.getShooterServo().setPosition(0.0);


//         }

//         private void initLogging()
//         {
//             TelemetryLogger.initLoggingSubsystem(this);
//             mLogger = new TelemetryLogger(this, "Op");

//             mLogger.info("Started logging");
//         }


//         /*
//          * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//          */
//         @Override
//         public void init_loop() {
//         }

//         /*
//          * Code to run ONCE when the driver hits PLAY
//          */
//         @Override
//         public void start() {
//             mTimer.reset();
//             //shooterPower = 1.0;
//         }
        
//         /*
//          * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//          */
//         @Override
//         public void loop()
//         {
//             // Chassis control:
//             loopTrolley();
//             // Intake and Shooter:
//             loopIntakeShooter();
//             // Wobble Arm and Grip
//             // loopWobble();

//             // Show the elapsed game time and wheel power.
//             //telemetry.addData("Status", "Run Time: " + mTimer.toString());
//         }

//         private void loopTrolley()
//         {
//             // Choose to drive using either Tank Mode, or POV Mode
//             // Comment out the method that's not used.  The default below is POV.

//             // POV Mode uses left stick to go forward, and right stick to turn.
//             // - This uses basic math to combine motions and is easier to drive straight.
//             double trolleyDrive = -gamepad1.left_stick_y;
//             double trolleyTurn  =  gamepad1.right_stick_x;
//             double trolleyStrafe = gamepad1.left_stick_x;
//             double maxTrolleyPower = 1.0;

            
//             double trolleyPower = maxTrolleyPower;
//             if(gamepad1.left_bumper)
//             {
//                 trolleyPower = 0.375;
//                 telemetry.addData("Parking mode", "ON");
//             }

//             //Not needed because the power is reset to 1 every loop
//             if(gamepad1.right_bumper)
//             {
//                 trolleyPower = maxTrolleyPower;
//                 telemetry.addData("Parking mode", "OFF");
//             }

//             double leftFrontPower    = Range.clip(trolleyDrive + trolleyTurn + trolleyStrafe, -trolleyPower, trolleyPower) ;
//             double leftBackPower = Range.clip(trolleyDrive + trolleyTurn - trolleyStrafe, -trolleyPower, trolleyPower) ;
//             double rightFrontPower   = Range.clip(trolleyDrive - trolleyTurn - trolleyStrafe, -trolleyPower, trolleyPower) ;
//             double rightBackPower = Range.clip(trolleyDrive - trolleyTurn + trolleyStrafe, -trolleyPower, trolleyPower) ;

            
//             if (trolleyDrive != 0 || trolleyTurn != 0 || trolleyStrafe != 0)
//             {
//                 mLogger.trace("Drive state: drive=" + trolleyDrive + ", turn=" + trolleyTurn + ", strafe=" + trolleyStrafe +
//                     ", LF=" + leftFrontPower + ", LB=" + leftBackPower +  ", RF=" + rightFrontPower + ", RB=" + rightBackPower);
//             }
            

//             // Tank Mode uses one stick to control each wheel.
//             // - This requires no math, but it is hard to drive forward slowly and keep straight.
//             // leftPower  = -gamepad1.left_stick_y ;
//             // rightPower = -gamepad1.right_stick_y ;

//             // Send calculated power to wheels
//             RobotDriveQuadMotor robotDrive = (RobotDriveQuadMotor)mRobot.getRobotDrive();

//             robotDrive.getFirstLeftDrive().setPower(leftFrontPower);
//             robotDrive.getLastLeftDrive().setPower(leftBackPower);
//             robotDrive.getFirstRightDrive().setPower(rightFrontPower);
//             robotDrive.getLastRightDrive().setPower(rightBackPower);


//         }
//         //creates variable that will set the intake motor power
//         double intakePower;
//         //creates variable that will set the shooter motor power
//         double shooterPower;
//         //creates variable that will assign loader arm position
//         double loaderPos;
        
//         private void loopIntakeShooter()
//         {
            
// //Intake and Shooter Logic
//             //intake on and Shooter off by default
//             intakePower = 1.0;
//             shooterPower = 0.0;
//             //Reverses intake for uncongesting
//             if(gamepad1.left_trigger==1)
//             {
//                 intakePower = -1.0;
                
//                 telemetry.addData("Intake", "REVERSE");
//             }
//             //Turns off intake and enables the Shooter if button pressed
//             else if(gamepad1.right_trigger==1)
//             {
//                 intakePower = 0.0;
//                 shooterPower = 1.0;
                
//                 telemetry.addData("Intake", "OFF");
//                 telemetry.addData("Shooter", "ON");
//             }
            
// //Loader Arm Logic
//             //Default loader arm position
//             loaderPos = 0.0;
//             //When button held, arm loads ring
//             if (gamepad1.b && gamepad1.right_trigger==1)
//             {
//                 mLogger.info("Firing");
//                 loaderPos = (1.0);
//             }
// //Sets Power and Position
//             //Sets the intake motor power
//             mRobot.mIntakeMotor.setPower(intakePower);
//             //Sets the shooter motor power
//             mRobot.mShooterMotor.setPower(shooterPower);
//             //Sets the loader arm servo position
//             mRobot.getShooterServo().setPosition(loaderPos);
//         }
//         /*private void loopWobble()
//         {
//             // slew the servo, according to the rampUp (direction) variable.
//             if (rampUp) {
//                 // Keep stepping up until we hit the max value.
//                 position += INCREMENT ;
//                 if (position >= MAX_POS ) {
//                     position = MAX_POS;
//                     rampUp = !rampUp;   // Switch ramp direction
//                 }
//             }
//             else {
//                 // Keep stepping down until we hit the min value.
//                 position -= INCREMENT ;
//                 if (position <= MIN_POS ) {
//                     position = MIN_POS;
//                     rampUp = !rampUp;  // Switch ramp direction
//                 }
//             }

//             // Display the current value
//             telemetry.addData("Servo Position", "%5.2f", position);
//             telemetry.addData(">", "Press Stop to end test." );
//             telemetry.update();

//             // Set the servo to the new position and pause;
//             servo.setPosition(position);
//             sleep(CYCLE_MS);
//             idle();
//         }*/



//         @Override
//         public void stop()
//         {
//             if (mDrive != null)
//             {
//                 mDrive.stopAll();
//             }
//         }
//     }
