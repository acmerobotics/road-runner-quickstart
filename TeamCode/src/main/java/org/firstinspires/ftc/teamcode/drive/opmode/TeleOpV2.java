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

    package org.firstinspires.ftc.teamcode.drive.opmode;

    import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.OldCode.TelemetryLogger;
    import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.PIDFCoefficients;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DigitalChannel;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;

    import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    /**
     * This file contains an example of an iterative (Non-Linear) "OpMode".
     * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
     * The names of OpModes appear on the menu of the FTC Driver Station.
     * When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all iterative OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @TeleOp(name="TeleOpV2", group="UltimateGoal")

    public class UltimateGoalTeleOpTest extends OpMode
    {
        /**
         * Hardware class implementation
         */
        SampleMecanumDrive drive;

        /**
         * Quad robot drive.
         */

        double targetVel;

        int tolerance = 100;

        boolean atSpeed = false;

        private TelemetryLogger mLogger;

        /**
         * Timer to monitor duration of the calibrated operations.
         */
        private ElapsedTime         mTimer;

        private DcMotorEx  mShooterMotorEx;

        @Override
        public void init()
        {
            initLogging();
            mLogger.info("Initializing");

            mTimer = new ElapsedTime();
            drive = new SampleMecanumDrive(hardwareMap);

            //Drive Train (Rest is taken care of in SampleMecanumDrive
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // In teleop mode, the drive will not be operated using encoders - set it to non-encoder mode.
            drive.setMode(RUN_WITHOUT_ENCODER);

            robot.getShooterServo().setPosition(0.0);

            mShooterMotorEx =(DcMotorEx)robot.mShooterMotor;
            if (mShooterMotorEx != null){
                telemetry.addLine("It works!");
            }

        }

        private void initLogging()
        {
            TelemetryLogger.initLoggingSubsystem(this);
            mLogger = new TelemetryLogger(this, "Op");

            mLogger.info("Started logging");
        }


        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
            telemetry.clear();
            telemetry.addData("Velocity of mShooterMotorEx", mShooterMotorEx.getVelocity());
            if (mShooterMotorEx != null){
                telemetry.addLine("It works!");
            }
        }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            mTimer.reset();
            //shooterPower = 1.0;
            // We show the log in oldest-newest order
            telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
            // We can control the number of lines shown in the log
            telemetry.log().setCapacity(6);
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop()
        {
            // Chassis control:
            loopTrolley();
            // Intake and Shooter:
            loopIntakeShooter();
            // Wobble Arm and Servo:
            loopWobble();

            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + mTimer.toString());
        }

        private void loopTrolley()
        {
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double trolleyDrive = -gamepad1.left_stick_y;
            double trolleyTurn  =  gamepad1.right_stick_x;
            double trolleyStrafe = gamepad1.left_stick_x;
            double maxTrolleyPower = 1.0;


            double trolleyPower = maxTrolleyPower;
            if(gamepad1.left_bumper)
            {
                trolleyPower = 0.375;
                telemetry.addData("Parking mode", "ON");
            }

            //Not needed because the power is reset to 1 every loop
            if(gamepad1.right_bumper)
            {
                trolleyPower = maxTrolleyPower;
                telemetry.addData("Parking mode", "OFF");
            }

            double leftFrontPower    = Range.clip(trolleyDrive + trolleyTurn + trolleyStrafe, -trolleyPower, trolleyPower) ;
            double leftBackPower = Range.clip(trolleyDrive + trolleyTurn - trolleyStrafe, -trolleyPower, trolleyPower) ;
            double rightFrontPower   = Range.clip(trolleyDrive - trolleyTurn - trolleyStrafe, -trolleyPower, trolleyPower) ;
            double rightBackPower = Range.clip(trolleyDrive - trolleyTurn + trolleyStrafe, -trolleyPower, trolleyPower) ;


            if (trolleyDrive != 0 || trolleyTurn != 0 || trolleyStrafe != 0)
            {
                mLogger.trace("Drive state: drive=" + trolleyDrive + ", turn=" + trolleyTurn + ", strafe=" + trolleyStrafe +
                        ", LF=" + leftFrontPower + ", LB=" + leftBackPower +  ", RF=" + rightFrontPower + ", RB=" + rightBackPower);
            }


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
//            RobotDriveQuadMotor robotDrive = (RobotDriveQuadMotor)mRobot.getRobotDrive();
//
//            robotDrive.getFirstLeftDrive().setPower(leftFrontPower);
//            robotDrive.getLastLeftDrive().setPower(leftBackPower);
//            robotDrive.getFirstRightDrive().setPower(rightFrontPower);
//            robotDrive.getLastRightDrive().setPower(rightBackPower);

            drive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);
        }
        //creates variable that will set the intake motor power
        double intakePower;
        //creates variable that will set the shooter motor power
        double shooterPower;
        //creates variable that will assign loader arm position
        double loaderPos;

        private void loopIntakeShooter()
        {

//Intake and Shooter Logic
            //intake on and Shooter off by default
            intakePower = 1.0;
            targetVel = 0.0;
            //Reverses intake for decongesting
            if(gamepad1.left_trigger==1)
            {
                intakePower = -1.0;

                telemetry.addData("Intake", "REVERSE");
            }
            //Turns off intake and enables the Shooter if button pressed
            else if(gamepad1.right_trigger==1)
            {
                intakePower = 0.0;
                targetVel = 2400;

                telemetry.addData("Intake", "OFF");
                telemetry.addData("Shooter", "ON");
            }

//Loader Arm Logic
            //Default loader arm position
            loaderPos = 0.0;

            //Detects if shooter is at speed
            atSpeed = (mShooterMotorEx.getVelocity() < (targetVel + tolerance)) && (mShooterMotorEx.getVelocity() > (targetVel - tolerance * 0.5));

            //When button held and at speed, arm loads ring (speed drops with fire, resetting arm. when it speeds up again, arm can go back, making it automatic)
            if (gamepad1.b && gamepad1.right_trigger==1 && atSpeed)
            {
                mLogger.info("Firing");
                loaderPos = (1.0);
            }
//Sets Power and Position
            //Sets the intake motor power
            robot.mIntakeMotor.setPower(intakePower);
            //Coefficients
            // mShooterMotorEx.setVelocityPIDFCoefficients(1.37, 0.14, 0.0, 13.65);
//             mShooterMotorEx.setVelocityPIDFCoefficients(1.32, 0.132, 0.0, 13.2);
            mShooterMotorEx.setVelocityPIDFCoefficients(0.0, 0.5, 0.0, 5.0); // effective values from testing
            //Sets the shooter motor power
            // mShooterMotorEx.setPower(.7);
//            mShooterMotorEx.setVelocity(1240);          //during Aledo qual, this caused the rings to skim the bottom of the goal most shots
            mShooterMotorEx.setVelocity(targetVel);
            //mShooterMotorEx.setPower(shooterPower);
                /*
                if (gamepad2.a && velocity > 999){
                    velocity -= change;
                }
                else if (gamepad2.b && velocity < 2300){
                    velocity += change;
                }
                */
            telemetry.clear();
            telemetry.addLine()
                    .addData("Velocity of Shooter Motor", mShooterMotorEx.getVelocity());
            telemetry.addLine()
                    .addData("Power of Shooter Motor", mShooterMotorEx.getPower());
            telemetry.update();
            //
            //mRobot.mShooterMotor.setPower(shooterPower);
            //Sets the loader arm servo position
            robot.getShooterServo().setPosition(loaderPos);
        }
        static final double MAX_POS     =  0.5;     // Maximum rotational position
        static final double MIN_POS     =  0.05;     // Minimum rotational position
        double gripPos = MAX_POS;
        boolean gripWobble = false;
        private void loopWobble()
        {
            robot.mWobbleArmMotor.setPower(-(gamepad2.left_stick_y));
            if(!gripWobble && gamepad2.a)
            {
                gripPos = MIN_POS; //closing wobble grip
                gripWobble = true;
            }
            else if (gripWobble && gamepad2.a)
            {
                gripPos = MAX_POS; //open wobble grip
                gripWobble = false;
            }
            robot.mWobbleArmServo.setPosition(gripPos);
        }



        @Override
        public void stop()
        {
            if (drive != null)
            {
                drive.stopMotors();
            }
        }
    }
