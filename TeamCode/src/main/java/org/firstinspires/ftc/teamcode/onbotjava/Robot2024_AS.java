package org.firstinspires.ftc.teamcode.onbotjava;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;

import java.util.*;

public class Robot2024_AS {
    HardwareMap hardwareMap;
    LinearOpMode opmode;
    DriveTrainMecanum_AS driveTrain;
    TeamIMU_AS imu;
    ArmMovement_AS arm;
    String status = "";
    AprilTagComponent_AS_AS aprilTagComponent;
    //OdometryComponent_AS odo;
    ArrayList<String> alerts = new ArrayList<>();
    
    List<RobotComponent_AS> robotComponents = new ArrayList<>();
    
    
    
    public Robot2024_AS(LinearOpMode opmode, boolean isAuto){
        this.hardwareMap = opmode.hardwareMap;
        this.opmode = opmode;
        imu = new TeamIMU_AS(this);
        driveTrain = new DriveTrainMecanum_AS(this);
        if(isAuto){
            imu.ignoreHeadingError=false;
            //driveTrain.isAuto=true;
        }
        arm = new ArmMovement_AS(this);
        aprilTagComponent = new AprilTagComponent_AS_AS(this);
        //odo = new OdometryComponent_AS(this);
    }
    
    public void loop(){
        alerts.clear();
        for (RobotComponent_AS component: robotComponents) {
            component.loop();
        }

        opmode.telemetry.addData("Robot", String.format("#components: %d", robotComponents.size()));

        opmode.telemetry.addData("Status", status);
        opmode.telemetry.addData("ALERTS", alerts.toString());
        for (RobotComponent_AS component: robotComponents) {
            component.doTelemetry(opmode.telemetry);
        }
        opmode.telemetry.update();
    }
    
    public void setStatus(String status){
        this.status = status;
        
    }
    
    public void alert(String message) {
        alerts.add(message);
    }
    
    public void teleopLoop(){
        for (RobotComponent_AS component: robotComponents) {
            component.teleopLoop(opmode.gamepad1, opmode.gamepad2);
        }
        for (RobotComponent_AS component: robotComponents) {
            component.loop();
        }
    }

    public void registerComponent(RobotComponent_AS component) {
        robotComponents.add(component);
    }
    
    public void sleep(int ms){
        long startT =  System.currentTimeMillis();
        long stopT = startT + ms;
        while (!opmode.isStopRequested() && System.currentTimeMillis()<stopT)
        {
            driveTrain.stop();
            setStatus(String.format("Sleeping: %.0fsec remaining", 
                1.0*(stopT-System.currentTimeMillis())/1000));
            loop();
        }
    }
    public Pose2D getPosition(){
        return new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0);
        //return odo.currentPosition;
    }
    public Pose2D getVelocity(){
        return new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.DEGREES,0);
        //return odo.currentVelocity;
    }
    public void waitForGamePad2X(String message){
        setStatus(String.format("Waiting for GP2.X: %s", message));
        while(!opmode.gamepad2.x){
            loop();
        }
         while(opmode.gamepad2.x){
            loop();
        }
    }
}

