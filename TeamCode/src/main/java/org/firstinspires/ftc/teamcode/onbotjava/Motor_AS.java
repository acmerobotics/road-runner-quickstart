package org.firstinspires.ftc.teamcode.onbotjava;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.robot.Robot;


public class Motor_AS extends RobotComponent_AS {
    String motorLabel;
    private DcMotor motor;
    int prevPos;
    int deltaPos = 0;
    long prevTime_ms;
    long speed_tps = 0;
    int reference;
    float speedHistory[] = new float[10];
    int numHistories = 0;
    // used to detect first command to stop vs future commands to stop
    // when in POSITION mode
    boolean motorIsStopped=true;

    public Motor_AS(Robot2024_AS robot, String motorLabel){
        super(robot);
        this.motor = hardwareMap.dcMotor.get(motorLabel);
        this.motorLabel = motorLabel;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Default mode
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevPos = motor.getCurrentPosition();
        prevTime_ms = System.currentTimeMillis();
        
    }
    public int getPositionError(){
            return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition());
    }
    public void setDirection(DcMotorSimple.Direction dir) {
        motor.setDirection(dir);
    }
    public void loop(){
        int curPos = getCurrentPosition();
        long curTime_ms = System.currentTimeMillis();
        deltaPos = curPos - prevPos;
        long deltaTime_ms = curTime_ms - prevTime_ms;
        speed_tps = 1000L*deltaPos/deltaTime_ms;
        prevPos = curPos;
        prevTime_ms = curTime_ms;
        
        speedHistory[numHistories % speedHistory.length] = speed_tps;
        numHistories++;
    }

    
    public void setModeToPosition() {
        setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public void setModeToVelocity(){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public boolean isBusy(){
        return motor.isBusy();
    }
    
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb)
    {
        motor.setZeroPowerBehavior(zpb);
    }
    
    public double getPower() {
        return motor.getPower();
    }
    
    public void setPower(double power) {
        motor.setPower(power);
    }
    
    public void stop(){
        if ( motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // Save motor position the first time stop is requested
            if (!motorIsStopped) {
                motor.setTargetPosition(motor.getCurrentPosition());
            }
        } else {
            motor.setPower(0);
        }
        motorIsStopped = true;
    }
    
    public void setTargetPosition(double position) {
        motor.setTargetPosition((int) position + reference);
        motorIsStopped = false;
    }
    
    public void updateTargetPosition(double positionChange) {
        setTargetPosition(motor.getCurrentPosition() + positionChange);
    }
    
    public int getTargetPosition() {
        return motor.getTargetPosition() - reference;
    }
    
    public int getCurrentPosition(){
        return motor.getCurrentPosition() - reference;
    }
    
    public int getPositionChange(){
        return deltaPos;
    }
    
    public void setModeToEncoderSpeed() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    
    public void setReference() {
        reference = motor.getCurrentPosition();
    }

    public void doTelemetry(Telemetry telemetry){
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            telemetry.addData(motorLabel, String.format("Pos: t=%d, cur: %5d, e: %+3d, tps: %+4.0f, p: %.0f%%",getTargetPosition(), prevPos, (getTargetPosition()-prevPos), getAverageSpeed_tps(), motor.getPower()*100));
        else if (motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER)
            telemetry.addData(motorLabel, String.format("Speed: %+.0f%%, tps: %+4.0f, pos: %5d", motor.getPower()*100, getAverageSpeed_tps(), prevPos));
        else 
            telemetry.addData(motorLabel, String.format("Pwr: %+.0f%% pos: %5d tps: %+4.0f", motor.getPower()*100, prevPos, getAverageSpeed_tps()));
    }
    public double getAverageSpeed_tps() {
        if (numHistories == 0)
            return 0;
        //int n = Math.min(numHistories, speedHistory.length);
        float sum = 0;
        for (int i =0; i < speedHistory.length; i++)
            sum += speedHistory[i];
        return sum/Math.min(numHistories, speedHistory.length);
    }
    
}
