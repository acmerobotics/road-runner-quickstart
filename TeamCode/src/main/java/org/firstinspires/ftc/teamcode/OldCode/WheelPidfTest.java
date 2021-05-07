package org.firstinspires.ftc.teamcode.OldCode;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
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
@Disabled
@TeleOp (name="WheelPidfTest", group="UltimateGoalTest")

public class WheelPidfTest extends OpMode {

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

    DcMotorEx mShooterMotorEx;
    double currentVelocity;
    double maxVelocity = 0.0;
    
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
            if ((mShooterMotorEx.getVelocity() < (targetVel + tolerance))&&(mShooterMotorEx.getVelocity() > (targetVel - tolerance))) {
                if (!timing){
                timing = true;
                startTime = mTimer.milliseconds();
                }
                else if (mTimer.milliseconds() - startTime > stableTime){
                    atSpeed = true;
                }
            }
            else {
                timing = false;
            }
            // mShooterMotorEx.setPower(1.0);
            // currentVelocity = mShooterMotorEx.getVelocity();
            
            // if (currentVelocity > maxVelocity) {
            //     maxVelocity = currentVelocity;
            // }
            // mShooterMotorEx.setVelocityPIDFCoefficients(0.0, 0.5, 0.0, 5.0); // Group G
            mShooterMotorEx.setVelocityPIDFCoefficients(0.0, 0.5, 1.0, 5.0); // Group _

            mShooterMotorEx.setVelocity(targetVel);
            
            telemetry.clear();
            telemetry.addLine()
                         .addData("Velocity of Shooter Motor", mShooterMotorEx.getVelocity());
            // telemetry.addLine()
            //              .addData("Power of Shooter Motor", mShooterMotorEx.getPower());
            telemetry.addData("Power of Shooter Motor", mShooterMotorEx.getPower());
            if (atSpeed){
                telemetry.addData("Time taken to target", startTime);
            }
            // telemetry.addData("current velocity", currentVelocity);
            // telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
}