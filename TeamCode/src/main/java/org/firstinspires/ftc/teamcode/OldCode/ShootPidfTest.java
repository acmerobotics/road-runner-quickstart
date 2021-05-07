package org.firstinspires.ftc.teamcode.OldCode;

    import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.PIDFCoefficients;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DigitalChannel;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;

@TeleOp (name="ShootPidfTest", group="UltimateGoalTest")

public class ShootPidfTest extends OpMode {

    /**
         * Hardware class implementation
         */
        UltimateGoalHardwareTest        mRobot;

        /**
         * Quad robot drive.
         */
        RobotDrive               mDrive;
        
        private TelemetryLogger  mLogger;

        /**
         * Timer to monitor duration of the calibrated operations.
         */
        private ElapsedTime         mTimer;
        
        private int tolerance;
        private int targetVel;
        private boolean timing;
        private double startTime;
        private double stableTime;
        private boolean atSpeed;
        private boolean upPressed;
        private boolean downPressed;

    DcMotorEx mShooterMotorEx;
    double currentVelocity;
    double minVelocity = 0.0;
    
    @Override
    public void init()
        {
            initLogging();
            mLogger.info("Initializing");

            mTimer = new ElapsedTime();
            mRobot = new UltimateGoalHardwareTest(this, mLogger);
            mRobot.init();
            //Shooter Motor
            mShooterMotorEx = (DcMotorEx)mRobot.mShooterMotor;
            
            tolerance = 100;  // Ticks per second
            targetVel = 2200;  // target velocity in ticks per second
            stableTime = 500;  // milliseconds 
        }
        
        private void initLogging()
        {
            TelemetryLogger.initLoggingSubsystem(this);
            mLogger = new TelemetryLogger(this, "Op");

            mLogger.info("Started logging");
        }
        
        
        @Override
        public void start() {
            mTimer.reset();
        }
        @Override
        public void loop() {
            //PIDF Timing Test
            if ((mShooterMotorEx.getVelocity() < (targetVel + tolerance*1.5))&&(mShooterMotorEx.getVelocity() > (targetVel - tolerance*0.5))) {
                if (!timing){
                timing = true;
                startTime = mTimer.milliseconds();
                }
                else if (mTimer.milliseconds() - startTime > stableTime){
                    atSpeed = true;
                    minVelocity = targetVel;
                }
            }
            else {
                timing = false;
                atSpeed = false;
            }
            
            currentVelocity = mShooterMotorEx.getVelocity();
            
            if (atSpeed && gamepad1.b)
            {
                mRobot.getShooterServo().setPosition(1.0);
                mTimer.reset();
            }
            else
            {
                mRobot.getShooterServo().setPosition(0.0);
            }
            
            if (!timing) {
                if (currentVelocity < minVelocity) {
                    minVelocity = currentVelocity;
                }
            }
            
            if (gamepad1.dpad_up && !upPressed)
            {
                targetVel += 100;
                upPressed = true;
            }
            else if (!gamepad1.dpad_up && upPressed)
            {
                upPressed = false;
            }
            
            if (gamepad1.dpad_down && !downPressed)
            {
                targetVel -= 100;
                downPressed = true;
            }
            else if (!gamepad1.dpad_down && downPressed)
            {
                downPressed = false;
            }
            
            mShooterMotorEx.setVelocityPIDFCoefficients(0.0, 0.5, 0.0, 5.0); // Group G
            // mShooterMotorEx.setVelocityPIDFCoefficients(0.0, 0.5, 1.0, 5.0); // Group _

            mShooterMotorEx.setVelocity(targetVel);
            
            telemetry.clear();
            telemetry.addLine()
                     .addData("Velocity of Shooter Motor", mShooterMotorEx.getVelocity());
            telemetry.addLine()
                     .addData("Power of Shooter Motor", mShooterMotorEx.getPower());
            telemetry.addLine()
                     .addData("Current Target Velocity", targetVel);
            if (atSpeed){
                telemetry.addData("Time taken to target", startTime);
            }
            if (!atSpeed){
                telemetry.addData("Minimum Velocity", minVelocity);
            }
            // telemetry.addData("current velocity", currentVelocity);
            // telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
}
