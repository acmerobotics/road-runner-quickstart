package org.firstinspires.ftc.teamcode.onbotjava;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Test wheel moters(names may be swapped in configeration)
public class ArmMovement extends RobotComponent{
    public final double CONVEYOR_INTAKE_POWER= -1.0;
    Motor intakeMotor, outtakeMotor;
    Servo outtakeServo;
    CRServo intakeServo;
    boolean liftWasPressed=false;
    boolean liftWasPressed2=false;
    boolean inFlag;
    boolean outFlag;
    //TouchSensor intakeSensor;
    CRServo conveyorServo;
    CRServo backIntakeServo;
    TouchSensor outtakeSensor;
    Motor leftClimberMotor;
    Motor rightClimberMotor;
    boolean climberHoldMode=false;
    
    TouchSensor leftClimberLimit, rightClimberLimit;
public ArmMovement(Robot2024 robot){
    super(robot);
    //intakeSensor = hardwareMap.get(TouchSensor.class, "intake_limit");
    outtakeSensor = hardwareMap.get(TouchSensor.class, "lift_limit");
    leftClimberLimit = hardwareMap.get(TouchSensor.class, "left_climb_limit");
    rightClimberLimit = hardwareMap.get(TouchSensor.class, "right_climb_limit");
    intakeMotor= new Motor(robot, "intake");
    outtakeMotor= new Motor(robot, "lift");
    leftClimberMotor = new Motor(robot,"left_climber");
    leftClimberMotor.setReference();
    rightClimberMotor = new Motor(robot,"right_climber");
    rightClimberMotor.setReference();
    
    rightClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightClimberMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    leftClimberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    outtakeMotor.setModeToPosition();
    //testing outake
    //outtakeMotor.setModeToVelocity();
    outtakeMotor.setPower(1.0);
    outtakeServo = hardwareMap.servo.get("dump");
    intakeServo = hardwareMap.crservo.get("intake_servo");
    outtakeToStart();
    intakeMotor.setReference();
    outtakeMotor.setReference();
    backIntakeServo = hardwareMap.crservo.get("backintake_servo");
    conveyorServo= hardwareMap.crservo.get("conveyor_servo");
    
}

public void loop() {
    if (outtakeSensor.isPressed())
        outtakeMotor.setReference();
        
    int forwardLimit = -1900;
    if (intakeMotor.getCurrentPosition() <= forwardLimit && intakeMotor.getPower() < 0) {
        robot.alert("Too far forward");
        intakeMotor.setPower(0);
    }
    if (!climberHoldMode) {
        if(Math.abs(leftClimberMotor.getPower()) > 0.15 && !canLeftClimberRetract()) {
            robot.alert("Stopping left climber");
            leftClimberMotor.setPower(0);
        }
        if(Math.abs(rightClimberMotor.getPower()) > 0.15 && !canRightClimberRetract()) {
            robot.alert("Stopping right climber");
            rightClimberMotor.setPower(0);
        }
    }
    
    if (leftClimberLimit.isPressed()){
        leftClimberMotor.setReference();
    }
    if (rightClimberLimit.isPressed()){
        rightClimberMotor.setReference();
    }
}
public void teleopLoop(Gamepad gamepad1, Gamepad gamepad2){
    if (gamepad1.b || climberHoldMode) {
        climberHoldMode=true;
        leftClimberIn(0.75);
        rightClimberIn(0.75);
    }
    
    if (!climberHoldMode) {
        //Climbers
        if (gamepad2.right_trigger>0.05){
            outFlag = true;
            inFlag = false;
            leftClimberOut(gamepad2.right_trigger);
            rightClimberOut(gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger>0.05){
            inFlag = true;
            outFlag = false;
            leftClimberIn(gamepad2.left_trigger);
            rightClimberIn(gamepad2.left_trigger);
        }
        else {
            // If neither trigger is pressed, then slowing pull the 
            // tape measure in based on the bumpers or stop them if
            // their bumper isn't pressed
            leftClimberIn(0);
            rightClimberIn(0);
        }
    }
    // INTAKE
    if (gamepad2.b&& !gamepad2.y){
        // Intake
        intakeServo.setPower(1.0);
        backIntakeServo.setPower(-1);
        
        // Turn on the conveyor when the intake is all the way in
        conveyorServo.setPower(0);
        
        
    }
    else if (gamepad2.b&&gamepad2.y){
        intakeServo.setPower(-1.0);
        backIntakeServo.setPower(-1);
        conveyorServo.setPower(CONVEYOR_INTAKE_POWER);
    }
    else if (gamepad2.x) {
        // OUTAKE
        intakeServo.setPower(1);
        conveyorServo.setPower(1);
        backIntakeServo.setPower(1);
    }
    else {
        // STOP INTAKE
        intakeServo.setPower(0);
        conveyorServo.setPower(0);
        backIntakeServo.setPower(0);

    }
    if (gamepad2.b) {
        outtakeToStart();
    }else if(gamepad2.a){
        //dump
        outtakeToDrop();
    }
    else if (gamepad2.left_bumper){
        //speciman pick up
        outtakeToFlat();
    }else{
        outtakeToCarry();
    }
    
    if (gamepad2.dpad_up){
        //lift
        liftWasPressed = true;
        outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition()-5000);//was -1500
    }
    else if (gamepad2.dpad_down){
        liftWasPressed = true;
        outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition()+500);
    } else if (Math.abs(gamepad2.right_stick_y)>0.1){
        liftWasPressed = true;
        outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition()+250*gamepad2.right_stick_y);
    }
    else{
        // Tell lift to stay in its current position, but do it just
        // when the up/down button is released
        if (liftWasPressed){
            outtakeMotor.setTargetPosition(outtakeMotor.getCurrentPosition());
        }
        liftWasPressed = false;
    }
    
    
    if (gamepad2.dpad_right){
        intakeMotor.setPower(-0.8);
    }
    else if(gamepad2.dpad_left){
        intakeMotor.setPower(0.8);
    } else {
        intakeMotor.setPower(0);
    }
    
    
}
public void leftClimberOut(double speed){
    leftClimberMotor.setPower(speed);
}
public void leftClimberIn(double speed){
    if(inFlag == true && !canLeftClimberRetract()){
        inFlag=false;
        robot.alert("Ignoring left climber input");
        return;
    } else
        leftClimberMotor.setPower(-speed);
}
public void rightClimberOut(double speed){
    rightClimberMotor.setPower(speed);
}
public void rightClimberIn(double speed){
    if(inFlag==true && !canRightClimberRetract()){
        inFlag=false;
        robot.alert("Ignoring right climber input");
        return;
    } else
        rightClimberMotor.setPower(-speed);
        
}
public boolean canRightClimberRetract() {
    if (rightClimberLimit.isPressed())
        return false;
        
    return true;
}

public boolean canLeftClimberRetract() {
    if (leftClimberLimit.isPressed())
        return false;
        
    return true;
}
public void outtakeToFlat(){
    outtakeServo.setPosition(0.3);
}
    
public void outtakeToStart(){
    outtakeServo.setPosition(0.6);
}
public void outtakeToCarry(){
    outtakeServo.setPosition(0.5);
}
public void outtakeToDrop(){
    outtakeServo.setPosition(0.2);
}
//height is in ticks negative is up posotive is down -4900 is near full height
public void setLiftHeight(long height_ticks, boolean wait){
    outtakeMotor.setTargetPosition(height_ticks);
    if (!wait)
        return;
    long currentPosition = outtakeMotor.getCurrentPosition();
    long error = height_ticks-currentPosition;
    while(!opmode.isStopRequested() && Math.abs(error) > 50){
        currentPosition = outtakeMotor.getCurrentPosition();
        error = height_ticks-currentPosition;
        robot.setStatus(String.format("Waiting for lift: e=%d, p=%d, g=%d",
            error, currentPosition, height_ticks));
        robot.loop();
    }
    robot.setStatus("Arm setHeight() done");
}
public void setLiftHeight(long height_ticks){
    setLiftHeight(height_ticks,true);
}
public void dropBlock(){
    robot.arm.outtakeToDrop();
    robot.sleep(2*1000);
    robot.arm.outtakeToStart();
}
public void setServoPowersToZero(){
    intakeServo.setPower(0);
    backIntakeServo.setPower(0);
    conveyorServo.setPower(0); 
}
public void doTelemetry(Telemetry telemetry) {
    telemetry.addData("Servo", "dump: %.2f intake:%.1f back_intake: %.1f, conveyor:%.1f",
            outtakeServo.getPosition(),intakeServo.getPower(),backIntakeServo.getPower(), conveyorServo.getPower());
    telemetry.addData("Arm_Pos","intakeTouch:N/A liftTouch:%s",
            outtakeSensor.isPressed()?"T":"F");
    telemetry.addData("Climber","right_climb_limit:%s left:%s",
            rightClimberLimit.isPressed()?"T":"F",leftClimberLimit.isPressed()?"T":"F");
}
}
