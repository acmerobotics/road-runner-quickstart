package org.firstinspires.ftc.teamcode.onbotjava;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;
import java.lang.Math;
import java.util.*;

public class Robot2024 {
    HardwareMap hardwareMap;
    LinearOpMode opmode;
    DriveTrainMecanum driveTrain;
    TeamIMU imu;
    ArmMovement arm;
    String status = "";
    AprilTagComponent aprilTagComponent;
    OdometryComponent odo;
    ArrayList<String> alerts = new ArrayList<>();
    
    List<RobotComponent> robotComponents = new ArrayList<>();
    
    
    
    public Robot2024(LinearOpMode opmode, boolean isAuto){
        this.hardwareMap = opmode.hardwareMap;
        this.opmode = opmode;
        imu = new TeamIMU(this);
        driveTrain = new DriveTrainMecanum(this);
        if(isAuto){
            imu.ignoreHeadingError=false;
            //driveTrain.isAuto=true;
        }
        arm = new ArmMovement(this);
        aprilTagComponent = new AprilTagComponent(this);
        odo = new OdometryComponent(this);
    }
    
    public void loop(){
        alerts.clear();
        for (RobotComponent component: robotComponents) {
            component.loop();
        }

        opmode.telemetry.addData("Robot", String.format("#components: %d", robotComponents.size()));

        opmode.telemetry.addData("Status", status);
        opmode.telemetry.addData("ALERTS", alerts.toString());
        for (RobotComponent component: robotComponents) {
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
        for (RobotComponent component: robotComponents) {
            component.teleopLoop(opmode.gamepad1, opmode.gamepad2);
        }
        for (RobotComponent component: robotComponents) {
            component.loop();
        }
    }

    public void registerComponent(RobotComponent component) {
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
        return odo.currentPosition;
    }
    public Pose2D getVelocity(){
        return odo.currentVelocity;
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

