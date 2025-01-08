package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;




public class DriveTrainMecanum extends RobotComponent {
    Motor frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

   long TICKS_PER_INCH = 181;
    // Copy some data from gamepad to be used in Telemetry
    double fo_x, fo_y, ro_x, ro_y;
    double spin;
    double foAngle_deg,roAngle_deg, drivePower;
    
    boolean prevX,prevB;
   
    public DriveTrainMecanum(Robot2024 robot) {
        super(robot);
       
        frontRightMotor = new Motor(robot, "FrontRight");
        backLeftMotor = new Motor(robot, "BackLeft");
        frontLeftMotor = new Motor(robot, "FrontLeft");
        backRightMotor = new Motor(robot, "BackRight") ;
        // for (Motor motor: motors) {
        //     motor.setModeToEncoderSpeed();
        //     motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // }   
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    

    public void loop() {
        
    }
    public long getLocation(){
        return Math.abs(frontLeftMotor.getCurrentPosition()) + 
        Math.abs(backLeftMotor.getCurrentPosition()) + Math.abs(frontRightMotor.getCurrentPosition()) +
        Math.abs(backRightMotor.getCurrentPosition());
    }
        
    private double getAngleFromJoystick (double x, double y)
    {
        double result= Math.atan2 (y,x)*180/Math.PI;
        //atan returns a number between -179 and +180 but we want between 
        //0 to 360
        
        if (result < 0)
            result += 360;
        return result; 
    }
    
    public void driveToAprilTagGoal(double maxSpeed, double goalX, double goalY) {
        
        boolean done=false;        

        while (!opmode.isStopRequested() && !done) {
            AprilTagDetection atDetection = robot.aprilTagComponent.currentDetection;
        
            if (atDetection == null ) {
                robot.setStatus("No april tag seen");
                robot.loop();
                continue;
            }
            //multiplying by -1 because negative angle are positive and positive angles are negative.
            double robotX =  -1 * atDetection.ftcPose.range * Math.sin(degreesToRadians(atDetection.ftcPose.bearing)) ;
            double robotY = atDetection.ftcPose.range * Math.cos(degreesToRadians(atDetection.ftcPose.bearing));
            
            double errorX = goalX - robotX;
            double errorY = goalY - robotY;
            double joyStickX = errorY;
            double joyStickY = -errorX;
            double errorDistance = Math.sqrt(errorX*errorX+errorY*errorY);
            robot.setStatus(String.format("Moving to AT. G:[%.0f, %.0f], L:[%.0f, %.0f], E: [%.0f, %.0f], GD:%.1f, J:[%.1f, %.1f]",
                goalX, goalY, robotX, robotY, errorX, errorY, errorDistance, joyStickX, joyStickY));
            done = errorDistance < 1 ;
            driveWithJoyStickAngles_robotOriented(maxSpeed,joyStickX ,joyStickY , getSpinPowerForHeadingCorrection());
            robot.loop();
        }
        robot.setStatus("Reached April Tag");
    }
    public double degreesToRadians(double degrees){
        return degrees * Math.PI/180;
    }
    public void teleopLoop(Gamepad gamepad1, Gamepad gamepad2) {
        double speedFactor=1.0;
        if (gamepad1.a)
            speedFactor=.25;
        if (!prevX && gamepad1.x) 
            robot.imu.updateDesiredHeading(90);
        else if (!prevB && gamepad1.b)
            robot.imu.updateDesiredHeading(-90);
        prevX = gamepad1.x;
        prevB = gamepad1.b;
        
        if (gamepad1.dpad_up)
            robot.imu.resetHeading_North();
        else if (gamepad1.dpad_left)
             robot.imu.resetHeading_West();
        else if (gamepad1.dpad_down)
             robot.imu.resetHeading_South();
        else if (gamepad1.dpad_right)
             robot.imu.resetHeading_East();

        fo_y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        fo_x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
        spin = gamepad1.right_stick_x;
        drivePower = Math.sqrt(fo_x*fo_x+fo_y*fo_y);
        driveWithJoyStickAngles_fieldOriented(drivePower*speedFactor,fo_x,fo_y,spin*speedFactor);


        if (gamepad1.right_bumper){
            shake();
        }
            
    }
    
    public void driveWithJoyStickAngles_fieldOriented(double drivePower,double fo_x_in,double fo_y_in,double spin){
         double fo_x=fo_x_in, fo_y=fo_y_in;
        // Normalize the x & y so they are maximum of -1...+1
        if (Math.abs(fo_x)>1 || Math.abs(fo_y) > 1) {
            double biggestMagnitude = Math.max(Math.abs(fo_x), Math.abs(fo_y));
            fo_x /= biggestMagnitude;
            fo_y /= biggestMagnitude;
        }
        robot.opmode.telemetry.addData("FO Drive Cmd", String.format("DrvP=%.2f, fo_in=[%.1f, %.1f], scaled_fo=[%.1f, %.1f]",
            drivePower, fo_x_in, fo_y_in, fo_x, fo_y));

        foAngle_deg=getAngleFromJoystick(fo_x,fo_y);
        drivePower=drivePower*Math.sqrt(fo_x*fo_x+fo_y*fo_y);
        roAngle_deg= foAngle_deg-robot.imu.getRobotAngle();
        ro_x=Math.cos(degreesToRadians(roAngle_deg));
        ro_y=Math.sin(degreesToRadians(roAngle_deg));
        
        //driveWithJoyStickAngles_robotOriented(drivePower,ro_x,ro_y,getSpinPowerForHeadingCorrection());
        driveWithJoyStickAngles_robotOriented(drivePower,ro_x,ro_y,spin);
    }
    
    /* this function will set motor powers based on x and y from joystick.
    the movement of the robot will be forward if y>0 and to the left if x<0.
    */
    public void driveWithJoyStickAngles_robotOriented(double drivePower,double ro_x_in, double ro_y_in,double spin){
        double ro_x=ro_x_in, ro_y=ro_y_in;
        // Normalize the x & y so they are maximum of -1...+1
        if (Math.abs(ro_x)>1 || Math.abs(ro_y) > 1) {
            double biggestMagnitude = Math.max(Math.abs(ro_x), Math.abs(ro_y));
            ro_x /= biggestMagnitude;
            ro_y /= biggestMagnitude;
        }
        
        robot.opmode.telemetry.addData("RO Drive Cmd", String.format("DrvP=%.2f, ro_in=[%.1f, %.1f], scaled_ro=[%.1f, %.1f]",
            drivePower, ro_x_in, ro_y_in, ro_x, ro_y));
        // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(ro_y) + Math.abs(ro_x) + Math.abs(spin), 1);
            double frontLeftPower =  (drivePower*(ro_y + ro_x) + spin) / denominator;
            double backLeftPower =   (drivePower*(ro_y - ro_x) + spin) / denominator;
            double frontRightPower = (drivePower*(ro_y - ro_x) - spin) / denominator;
            double backRightPower =  (drivePower*(ro_y + ro_x) - spin) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
    }
            
    public void moveInDirectForTime(double speed, double x, double y, long ms){
        long startT=System.currentTimeMillis();
        double spinPower=0;
        while (!opmode.isStopRequested() && System.currentTimeMillis()-startT<ms)
        {
            spinPower=getSpinPowerForHeadingCorrection();
                
            driveWithJoyStickAngles_robotOriented(speed, x, y, spinPower );
            robot.loop();
    
        }
    }
    
    public void moveInDirectForDistance(double speed, double x, double y, double inches){
        frontLeftMotor.setReference();
        backLeftMotor.setReference();
        frontRightMotor.setReference();
        backRightMotor.setReference();
        long endP=(long)(inches*TICKS_PER_INCH);
        double spinPower=0;
        while (!opmode.isStopRequested() && getLocation()<endP)
        {
            robot.setStatus(String.format("MovingForDistance. %d ticks to go", endP-getLocation()));
            spinPower=getSpinPowerForHeadingCorrection();
                
            driveWithJoyStickAngles_robotOriented(speed, x, y, spinPower );
            robot.loop();
    
        }
        robot.setStatus(String.format("MovingForDistance. Stopping"));
        stop();
    }
    
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("DriveCtl", "J: %3.1f, %3.1f = %3.0fd(FO)/%3.0f%%, Spin: %+3.0f%%", fo_x, fo_y, foAngle_deg, drivePower*100, spin*100);
        telemetry.addData("Drive Angles", "FO Drive Angle: %5.1f, RO Drive Angle: %5.1f... [ro_x,ro_y]=[%.1f, %.1f]", foAngle_deg, roAngle_deg, ro_x, ro_y);
        telemetry.addData("location", "%d", getLocation());
        //telemetry.addData("DriveTrain", "Dir=%3.0fd, Pwr=%3.0f%%, SpinPwr=%3.0f%%", moveInRobotDirection, movementPower*100, spinPower*100 );
    }
    
      public double getCurrentPosition ()
    {
        return frontRightMotor.getCurrentPosition();
    }
    public double getPositionChange ()
    {
        return frontLeftMotor.getPositionChange();
    }
    
    public void shake(){
        long current_time_ms=System.currentTimeMillis();
        long frequency_ms=200;
        
        // This will be 0->399
        long partOfCycle = current_time_ms % frequency_ms;
        
        if ( partOfCycle < frequency_ms/2) {
            driveWithJoyStickAngles_robotOriented(1.0,0,-1,0);

        } else {
            driveWithJoyStickAngles_robotOriented(1.0,0,1,0);
        }
    }
    
    public void stop()
    {
        frontLeftMotor.stop();
        backLeftMotor.stop();
        frontRightMotor.stop();
        backRightMotor.stop();
    }
    public double getSpinPowerForHeadingCorrection(){
        double minimalPowerToSpin = 0.20;
        double correctionPowerPerDegree=-0.005;
        double p=robot.imu.getHeadingError()*correctionPowerPerDegree;
        if (Math.abs(p)>minimalPowerToSpin)
            return p;
        else if (Math.abs(robot.imu.getHeadingError())>0.5)
            return Math.copySign(minimalPowerToSpin,p);
        else
            return 0;
        
    }
    public double getSpeedForDistanceCorrection(Pose2D targetPosition){
        double speedCorrectionPerInch=0;
        Pose2D currentPosition=robot.getPosition();
        double yError=targetPosition.getY(DistanceUnit.INCH)-currentPosition.getY(DistanceUnit.INCH);
        double xError=targetPosition.getX(DistanceUnit.INCH)-currentPosition.getX(DistanceUnit.INCH);
        double s=Math.sqrt((xError*xError)+(yError*yError))*speedCorrectionPerInch;
        return s;
    }
    
    public void turnCCW(double degrees) {
        turn (degrees);
    }
    
    public void turnCW(double degrees) {
        turn(-degrees);
    }
    
    // Positive degrees: CounterClockwise
    public void turn(double degrees){
        robot.imu.addToDesiredHeading(degrees);
        while (!opmode.isStopRequested() && Math.abs(robot.imu.getHeadingError())>1)
        {
            double spinPower=getSpinPowerForHeadingCorrection();
            robot.setStatus(String.format("Turning %.0f deg, %.1f deg to go [%.1f%% pwr]", degrees,robot.imu.getHeadingError(), spinPower));
            driveWithJoyStickAngles_robotOriented(0, 0, 0, spinPower );
            robot.loop();
    
        }
    }
    
    public double getDistanceBetweenPoses(DistanceUnit units, Pose2D pose1, Pose2D pose2) {
        double yError=pose1.getY(units)-pose2.getY(units);
        double xError=pose1.getX(units)-pose2.getX(units);
        return Math.sqrt(yError*yError+xError*xError);
    }
    public void moveToPosition(double topSpeed,Pose2D targetPosition){
        Pose2D currentPosition=robot.getPosition();
        Pose2D currentVelocity=robot.getVelocity();
        double distance = getDistanceBetweenPoses(DistanceUnit.INCH,targetPosition,currentPosition);
        robot.imu.setDesiredHeading(targetPosition.getHeading(AngleUnit.DEGREES));
        while (!opmode.isStopRequested() && (distance>2 || robot.imu.getHeadingError()>1))
        {
            currentPosition=robot.getPosition();
            distance = getDistanceBetweenPoses(DistanceUnit.INCH,targetPosition,currentPosition);
            double yError=targetPosition.getY(DistanceUnit.INCH)-currentPosition.getY(DistanceUnit.INCH);
            double xError=targetPosition.getX(DistanceUnit.INCH)-currentPosition.getX(DistanceUnit.INCH);
            double spinPower = getSpinPowerForHeadingCorrection();
            driveWithJoyStickAngles_fieldOriented(topSpeed,xError,yError,spinPower);

            
            robot.setStatus(
                
                String.format("Heading from [%.1f,%.1f] --> [%.1f, %.1f] (d=%.1f) (V=%.2f,%.2f)",
                    currentPosition.getX(DistanceUnit.INCH),
                    currentPosition.getY(DistanceUnit.INCH),
                    targetPosition.getX(DistanceUnit.INCH),
                    targetPosition.getY(DistanceUnit.INCH),
                    distance,
                    currentVelocity.getX(DistanceUnit.INCH),
                    currentVelocity.getY(DistanceUnit.INCH)
                )
            );
            
            
            robot.loop();
        }
        stop();
        robot.sleep(500);
        robot.setStatus("Reached target position");
    }
}
