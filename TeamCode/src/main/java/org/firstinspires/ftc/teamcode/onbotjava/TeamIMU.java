package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.robot.Robot;
import java.util.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class TeamIMU extends RobotComponent{
    final BNO055IMU imu;
    double previousDirection=0;
    double totalDegreesTurned=0;
    double desiredTotalDegreesTurned=0;
    boolean ignoreHeadingError = true;
    
    List<Double> pastHeadingErrors = new ArrayList<>();

    public TeamIMU(Robot2024 robot){
        super(robot);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        _getAngle1FromImu();
        totalDegreesTurned=desiredTotalDegreesTurned=90;
    }
    private float _getAngle1FromImu() {
        return imu.getAngularOrientation().firstAngle;
    }

    private float _getAngle2FromImu() {
        return imu.getAngularOrientation().secondAngle;
    }
    private float _getAngle3FromImu() {
        return imu.getAngularOrientation().thirdAngle;
    }
    
    public void updateDesiredHeading(double angle) {
        desiredTotalDegreesTurned += angle;
    }
    
    // [0,360) where 0 is 'North', 90 is 'West', 180 is 'South' and 270 is 'East'
    // North is defined to be the direction the robot was facing when it started
    // or reset to
    public double getHeading() {
        double heading = totalDegreesTurned % 360;
        if ( heading>=0 )
            return heading;
        else
            return heading+360;
    }
    
    public double getHeadingError(){
        if (ignoreHeadingError)
            return 0;
        else
            return desiredTotalDegreesTurned-totalDegreesTurned;
    }
    
    public double getAccumulatedHeadingError() {
        double result=0;
        for (Double e : pastHeadingErrors) {
            result += e;
        }
        return result;
    }
    
    public void resetHeading_North() {
        totalDegreesTurned=desiredTotalDegreesTurned = 90;
    }
    
    public void resetHeading_South() {
        totalDegreesTurned=desiredTotalDegreesTurned = 270;
    }
    
    public void resetHeading_East() {
        totalDegreesTurned=desiredTotalDegreesTurned = 0;
    }
    
    public void resetHeading_West() {
        totalDegreesTurned=desiredTotalDegreesTurned = 180;
    }
    
    public void doTelemetry(Telemetry telemetry) {
        telemetry.addData("IMU", getImuTelemetry());
    }
    public String getImuTelemetry() {
        return String.format("Tot=%+5.0f|Hdg=%3.0f|Tgt: %+5.0f|Raw=%3.0f",
            totalDegreesTurned,
            getHeading(),
            desiredTotalDegreesTurned,
            _getAngle1FromImu());
    }

    public double getRobotAngle() {
        return _getAngle1FromImu();
        
    }

    public void loop(){
        double currentDirection = _getAngle1FromImu();
        // TODO:
        // ... use newDirection and previousDirection and wrap-around
        // to update totalDegreesTurned
        double degChange = currentDirection - previousDirection;
        if(degChange > 180)
            degChange = degChange - 360;
        else if(degChange < -180)
            degChange = degChange + 360;
        totalDegreesTurned = totalDegreesTurned + degChange;
        previousDirection = currentDirection;
        
        double currentError = getHeadingError();
        
        // Dump our error history if the errors have changed sign
        if (pastHeadingErrors.size()>0) {
            double previousError = pastHeadingErrors.get(pastHeadingErrors.size()-1);
            
            if ( Math.copySign(1, currentError) != Math.copySign(1, previousError) )
                pastHeadingErrors.clear();
        }
        
        pastHeadingErrors.add(currentError);
        
        // Only keep 100 errors
        if (pastHeadingErrors.size()> 100 )
            // Remove oldest one
            pastHeadingErrors.remove(0);
    }
    
    public void addToDesiredHeading(double degrees) {
        desiredTotalDegreesTurned+=degrees;
        pastHeadingErrors.clear();
    }
    public void setDesiredHeading(double degrees){
        desiredTotalDegreesTurned = degrees;
        pastHeadingErrors.clear();
    }
    
    public void teleopLoop(Gamepad gamepad1, Gamepad gamepad2){
    }
}
