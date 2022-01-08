package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.firstinspires.ftc.teamcode.hardware.VelocityPIDFController;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


public class MotorPIDF extends Mechanism {
    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID;

    // Copy your feedforward gains here
    public static double kV = 0.00000;
    public static double kA = 0.00000;
    public static double kStatic = 0.00000;
    private String name;
    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    DcMotorEx motor;
    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
    private boolean running = false;
    public double targetVelo = 0;

    public MotorPIDF(String name, double kV, double kA, double kStatic, double kP, double kI, double kD){
        this.name = name;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);
    }

    public void init (HardwareMap hwMap){
        motor = hwMap.get(DcMotorEx.class, name);

        // Reverse as appropriate
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that RUN_USING_ENCODER is not set
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void update(){
        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = motor.getCurrentPosition();
        double motorVelo = motor.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);
        motor.setPower(power);
    }
    public void reset(){
        veloTimer.reset();
    }
    //    should be deprecated since update() fixes power
//    public void runEqual(double power) {
//        left.setPower(power);
//        right.setPower(power);
//    }
//    public void toggle(){
//        if(!running){
//            runEqual(1.0);
//        }
//
//        else{
//            runEqual(0.0);
//        }
//
//        running = !running;
//    }
    public boolean running(){
        return running;
    }
    public double veloc(){
        return motor.getVelocity();
    }
    public void setTargetVelo(double newVelo){
        targetVelo = newVelo;
    }
    public void toggle(double speed){
        if(!running){
            setTargetVelo(speed);
        }

        else{
            setTargetVelo(0.0);
        }

        running = !running;
    }
}