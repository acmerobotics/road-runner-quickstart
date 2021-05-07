package org.firstinspires.ftc.teamcode.OldCode;

    import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.PIDFCoefficients;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DigitalChannel;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;

@TeleOp (name="WheelVelocityTest", group="UltimateGoalTest")

public class MaxShooterWheelVelocity extends OpMode {

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
        

    DcMotorEx shooterMotorEX;
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
            shooterMotorEX = (DcMotorEx)mRobot.mShooterMotor;
            
        }
        
        private void initLogging()
        {
            TelemetryLogger.initLoggingSubsystem(this);
            mLogger = new TelemetryLogger(this, "Op");

            mLogger.info("Started logging");
        }
        @Override
    public void loop() {
            shooterMotorEX.setPower(1.0);
            currentVelocity = shooterMotorEX.getVelocity();
            
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            telemetry.clear();
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
}