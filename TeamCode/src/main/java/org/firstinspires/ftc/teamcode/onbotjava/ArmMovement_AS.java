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

//Test wheel moters(names may be swapped in configeration)
public class ArmMovement_AS extends RobotComponent_AS {
    public final double CONVEYOR_INTAKE_POWER= -1.0;
    Motor_AS intakeMotorAS, outtakeMotorAS;
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
    Motor_AS leftClimberMotorAS;
    Motor_AS rightClimberMotorAS;
    boolean climberHoldMode=false;
    
    TouchSensor leftClimberLimit, rightClimberLimit;
public ArmMovement_AS(Robot2024_AS robot){
    super(robot);
    //intakeSensor = hardwareMap.get(TouchSensor.class, "intake_limit");
    outtakeSensor = hardwareMap.get(TouchSensor.class, "lift_limit");
    leftClimberLimit = hardwareMap.get(TouchSensor.class, "left_climb_limit");
    rightClimberLimit = hardwareMap.get(TouchSensor.class, "right_climb_limit");
    intakeMotorAS = new Motor_AS(robot, "intake");
    outtakeMotorAS = new Motor_AS(robot, "lift");
    leftClimberMotorAS = new Motor_AS(robot,"left_climber");
    leftClimberMotorAS.setReference();
    rightClimberMotorAS = new Motor_AS(robot,"right_climber");
    rightClimberMotorAS.setReference();
    
    rightClimberMotorAS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightClimberMotorAS.setDirection(DcMotorSimple.Direction.REVERSE);

    leftClimberMotorAS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotorAS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    outtakeMotorAS.setModeToPosition();
    //testing outake
    //outtakeMotor.setModeToVelocity();
    outtakeMotorAS.setPower(1.0);
    outtakeServo = hardwareMap.servo.get("dump");
    intakeServo = hardwareMap.crservo.get("intake_servo");
    outtakeToStart();
    intakeMotorAS.setReference();
    outtakeMotorAS.setReference();
    backIntakeServo = hardwareMap.crservo.get("backintake_servo");
    conveyorServo= hardwareMap.crservo.get("conveyor_servo");
    
}

public void loop() {
    if (outtakeSensor.isPressed())
        outtakeMotorAS.setReference();
        
    int forwardLimit = -1900;
    if (intakeMotorAS.getCurrentPosition() <= forwardLimit && intakeMotorAS.getPower() < 0) {
        robot.alert("Too far forward");
        intakeMotorAS.setPower(0);
    }
    if (!climberHoldMode) {
        if(Math.abs(leftClimberMotorAS.getPower()) > 0.15 && !canLeftClimberRetract()) {
            robot.alert("Stopping left climber");
            leftClimberMotorAS.setPower(0);
        }
        if(Math.abs(rightClimberMotorAS.getPower()) > 0.15 && !canRightClimberRetract()) {
            robot.alert("Stopping right climber");
            rightClimberMotorAS.setPower(0);
        }
    }
    
    if (leftClimberLimit.isPressed()){
        leftClimberMotorAS.setReference();
    }
    if (rightClimberLimit.isPressed()){
        rightClimberMotorAS.setReference();
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
        outtakeMotorAS.setTargetPosition(outtakeMotorAS.getCurrentPosition()-5000);//was -1500
    }
    else if (gamepad2.dpad_down){
        liftWasPressed = true;
        outtakeMotorAS.setTargetPosition(outtakeMotorAS.getCurrentPosition()+500);
    } else if (Math.abs(gamepad2.right_stick_y)>0.1){
        liftWasPressed = true;
        outtakeMotorAS.setTargetPosition(outtakeMotorAS.getCurrentPosition()+250*gamepad2.right_stick_y);
    }
    else{
        // Tell lift to stay in its current position, but do it just
        // when the up/down button is released
        if (liftWasPressed){
            outtakeMotorAS.setTargetPosition(outtakeMotorAS.getCurrentPosition());
        }
        liftWasPressed = false;
    }
    
    
    if (gamepad2.dpad_right){
        intakeMotorAS.setPower(-0.8);
    }
    else if(gamepad2.dpad_left){
        intakeMotorAS.setPower(0.8);
    } else {
        intakeMotorAS.setPower(0);
    }
    
    
}
public void leftClimberOut(double speed){
    leftClimberMotorAS.setPower(speed);
}
public void leftClimberIn(double speed){
    if(inFlag == true && !canLeftClimberRetract()){
        inFlag=false;
        robot.alert("Ignoring left climber input");
        return;
    } else
        leftClimberMotorAS.setPower(-speed);
}
public void rightClimberOut(double speed){
    rightClimberMotorAS.setPower(speed);
}
public void rightClimberIn(double speed){
    if(inFlag==true && !canRightClimberRetract()){
        inFlag=false;
        robot.alert("Ignoring right climber input");
        return;
    } else
        rightClimberMotorAS.setPower(-speed);
        
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
    outtakeMotorAS.setTargetPosition(height_ticks);
    if (!wait)
        return;
    long currentPosition = outtakeMotorAS.getCurrentPosition();
    long error = height_ticks-currentPosition;
    while(!opmode.isStopRequested() && Math.abs(error) > 50){
        currentPosition = outtakeMotorAS.getCurrentPosition();
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
